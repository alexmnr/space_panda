#include "space_panda_controller/space_panda_controller.hpp"

#include <stddef.h>
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace space_panda_controller
{
  SpacePandaController::SpacePandaController() : controller_interface::ControllerInterface() {}

  ////////////////////// on_init /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_init() {
    // Declare Parameters
    try
    {
      auto_declare<std::string>("move_group", "");
      auto_declare<std::string>("tip_link", "");
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  ////////////////////// on_configure /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_configure(const rclcpp_lifecycle::State &) {
    // Read Parameters
    move_group_ = get_node()->get_parameter("move_group").as_string();
    if (move_group_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'move_group' parameter can't be empty!");
      return controller_interface::CallbackReturn::ERROR;
    }
    tip_link_ = get_node()->get_parameter("tip_link").as_string();
    if (tip_link_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'tip_link' parameter can't be empty!");
      return controller_interface::CallbackReturn::ERROR;
    }
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter can't be empty!");
      return controller_interface::CallbackReturn::ERROR;
    }
    else {
      num_joints_ = joint_names_.size();
    }

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

    // --- MoveIt Config ---
    auto model_node = std::make_shared<rclcpp::Node>("moveit_loader_node", get_node()->get_node_options());
    model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(model_node, "robot_description");
    robot_model_ = model_loader_->getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load RobotModel from parameter server.");
      return CallbackReturn::ERROR;
    }
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    joint_model_group_ = robot_model_->getJointModelGroup(move_group_); 
    tip_link_model_ = robot_state_->getLinkModel(tip_link_); 
    // Pre-allocate matrices
    jacobian_.resize(6, num_joints_);
    current_joint_positions_.resize(num_joints_, 0.0);
    RCLCPP_INFO(get_node()->get_logger(), "MoveIt Kinematics configured successfully.");

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
    return CallbackReturn::SUCCESS;
  }

  ////////////////////// update /////////////////////////
  controller_interface::return_type SpacePandaController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    // Read current joint positions from state interfaces
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      current_joint_positions_[i] = state_interfaces_[i].get_optional().value_or(0.0);
    }
    // Update MoveIt Kinematic State
    robot_state_->setJointGroupPositions(joint_model_group_, current_joint_positions_);
    robot_state_->updateLinkTransforms();
    // Compute the Jacobian at the tip link
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    robot_state_->getJacobian(joint_model_group_, tip_link_model_, reference_point, jacobian_);
    // Safely read the desired wrench from the Realtime Buffer
    Eigen::VectorXd desired_wrench_ee = Eigen::VectorXd::Zero(6); 
    auto command = rt_command_ptr_.readFromRT();
    if (command && *command) {
        desired_wrench_ee(0) = (*command)->wrench.force.x;
        desired_wrench_ee(1) = (*command)->wrench.force.y;
        desired_wrench_ee(2) = (*command)->wrench.force.z;
        desired_wrench_ee(3) = (*command)->wrench.torque.x;
        desired_wrench_ee(4) = (*command)->wrench.torque.y;
        desired_wrench_ee(5) = (*command)->wrench.torque.z;
    }
    // Get the transform of the tip_link relative to the root frame
    const Eigen::Isometry3d& link_transform = robot_state_->getGlobalLinkTransform(tip_link_model_);
    Eigen::Matrix3d rotation_matrix = link_transform.rotation();
    // Rotate the End-Effector wrench into the Base Frame
    Eigen::VectorXd desired_wrench_base = Eigen::VectorXd::Zero(6);
    // Apply rotation to the Force (first 3 elements)
    desired_wrench_base.head<3>() = rotation_matrix * desired_wrench_ee.head<3>();
    // Apply rotation to the Torque (last 3 elements)
    desired_wrench_base.tail<3>() = rotation_matrix * desired_wrench_ee.tail<3>();
    // Calculate required joint efforts: tau = J^T * F
    Eigen::VectorXd commanded_torques = jacobian_.transpose() * desired_wrench_base;
    // Write the torques to the command interfaces
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
    for (const std::string& joint_name : joint_names_) {
      names_.push_back(joint_name + "/position");
    }
    config.names = names_;
    return config;
  }

}  // namespace space_panda_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(space_panda_controller::SpacePandaController, controller_interface::ControllerInterface)
