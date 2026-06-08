#include "space_panda_controller/space_panda_controller.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace space_panda_controller
{
  SpacePandaController::SpacePandaController() : controller_interface::ControllerInterface() {}

  ////////////////////// on_init /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_init() {
    // Declare Parameters
    try
    {
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
      auto_declare<double>("command_force_limit", 5.0);
      auto_declare<double>("command_torque_limit", 0.5);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    
    const std::string robot_name_ = "panda";
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        robot_name_ + "/robot_model",
        robot_name_ + "/robot_state"
        );
    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
        robot_name_ + "/robot_state",
        robot_name_
        );
    return CallbackReturn::SUCCESS;
  }

  ////////////////////// on_configure /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_configure(const rclcpp_lifecycle::State &) {
    // Read Parameters
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter can't be empty!");
      return controller_interface::CallbackReturn::ERROR;
    }
    command_force_limit_ = get_node()->get_parameter("command_force_limit").as_double();
    command_torque_limit_ = get_node()->get_parameter("command_torque_limit").as_double();

    // --- Configuring Panda Force Thresholds ---
    RCLCPP_INFO(get_node()->get_logger(), "Configuring Panda Force Thresholds...");
    // Creating Service
    auto temp_panda_client_node_ = std::make_shared<rclcpp::Node>("temp_panda_client_node");
    panda_client_ = temp_panda_client_node_->create_client<franka_msgs::srv::SetForceTorqueCollisionBehavior>(
        "param_service_server/set_force_torque_collision_behavior"
        );
    while (!panda_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_node()->get_logger(), "Waiting for service: param_service_server/set_force_torque_collision_behavior");
    }

    // Creating Request
    auto request = std::make_shared<franka_msgs::srv::SetForceTorqueCollisionBehavior::Request>();
    request->lower_torque_thresholds_nominal = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
    request->upper_torque_thresholds_nominal = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
    request->lower_force_thresholds_nominal = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    request->upper_force_thresholds_nominal = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

    // Sending Request
    auto result = panda_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(temp_panda_client_node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Successfully configuring Panda Force Thresholds.");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to call service param_service_server/set_force_torque_collision_behavior");
      return CallbackReturn::FAILURE;
    }

    // --- Input Topic ---
    wrench_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "space_panda_controller/command_wrench", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            rt_command_ptr_.writeFromNonRT(msg);
        }
    );
    RCLCPP_INFO(get_node()->get_logger(), "Wrench subscriber configured.");
    return CallbackReturn::SUCCESS;
  }

  ////////////////////// on_activate /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_activate(const rclcpp_lifecycle::State &) {
    // Assign the loaned interfaces
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
    franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
    franka_robot_model_->initialize();
    return CallbackReturn::SUCCESS;
  }

  ////////////////////// update /////////////////////////
  controller_interface::return_type SpacePandaController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/){
    auto jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    // 2. Safely read the desired wrench from the Realtime Buffer
    Eigen::VectorXd desired_wrench_ee = Eigen::VectorXd::Zero(6); 
    auto command = rt_command_ptr_.readFromRT();
    if (command && *command) {
        rclcpp::Time msg_time((*command)->header.stamp);
        rclcpp::Duration msg_age = time - msg_time;
        // Apply the command only if it is newer than 100ms
        if (msg_age.seconds() <= 0.1) {
            desired_wrench_ee(0) = std::clamp((*command)->wrench.force.x,  -command_force_limit_,  command_force_limit_);
            desired_wrench_ee(1) = std::clamp((*command)->wrench.force.y,  -command_force_limit_,  command_force_limit_);
            desired_wrench_ee(2) = std::clamp((*command)->wrench.force.z,  -command_force_limit_,  command_force_limit_);
            desired_wrench_ee(3) = std::clamp((*command)->wrench.torque.x, -command_torque_limit_, command_torque_limit_);
            desired_wrench_ee(4) = std::clamp((*command)->wrench.torque.y, -command_torque_limit_, command_torque_limit_);
            desired_wrench_ee(5) = std::clamp((*command)->wrench.torque.z, -command_torque_limit_, command_torque_limit_);
        }
    }
    // 3. Fetch Native End-Effector Pose (std::array<double, 16>)
    auto current_pose_array = franka_robot_model_->getPose(franka::Frame::kEndEffector);
    // Map it to a 4x4 matrix and extract the 3x3 rotation matrix
    Eigen::Map<const Eigen::Matrix4d> current_pose(current_pose_array.data());
    Eigen::Matrix3d rotation_matrix = current_pose.block<3, 3>(0, 0);

    // 4. Rotate the End-Effector wrench into the Base Frame
    Eigen::VectorXd desired_wrench_base = Eigen::VectorXd::Zero(6);
    desired_wrench_base.head<3>() = rotation_matrix * desired_wrench_ee.head<3>();
    desired_wrench_base.tail<3>() = rotation_matrix * desired_wrench_ee.tail<3>();

    // 5. Calculate required joint efforts: tau = J^T * F
    Eigen::VectorXd commanded_torques = jacobian.transpose() * desired_wrench_base;

    // 6. Write the torques to the command interfaces
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      if (!command_interfaces_[i].set_value(commanded_torques(i))) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, 
                             "Failed to set effort command interface value for joint %zu", i);
      }
    }
    return controller_interface::return_type::OK;
  }

  ////////////////////// on_deactivate /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_deactivate(const rclcpp_lifecycle::State &) {
    franka_robot_state_->release_interfaces();
    franka_robot_model_->release_interfaces();
    return CallbackReturn::SUCCESS;
  }

  ////////////////////// command_interface_configuration /////////////////////////
  controller_interface::InterfaceConfiguration SpacePandaController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    std::vector<std::string> names_;
    for (const std::string& joint_name : joint_names_) {
      names_.push_back(joint_name + "/effort");
    }
    config.names = names_;
    return config;
  }

  ////////////////////// state_interface_configuration /////////////////////////
  controller_interface::InterfaceConfiguration SpacePandaController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    std::vector<std::string> names_;
    auto model_interfaces = franka_robot_model_->get_state_interface_names();
    names_.insert(names_.end(), model_interfaces.begin(), model_interfaces.end());
    config.names = names_;
    return config;
  }

}  // namespace space_panda_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(space_panda_controller::SpacePandaController, controller_interface::ControllerInterface)
