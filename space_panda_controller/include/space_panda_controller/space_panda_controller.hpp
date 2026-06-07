#ifndef SPACE_PANDA_CONTROLLER__SPACE_PANDA_CONTROLLER_HPP_
#define SPACE_PANDA_CONTROLLER__SPACE_PANDA_CONTROLLER_HPP_

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace space_panda_controller {
  class SpacePandaController : public controller_interface::ControllerInterface {
    public:
      SpacePandaController();
      controller_interface::CallbackReturn on_init() override;
      controller_interface::InterfaceConfiguration command_interface_configuration() const override;
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;
      controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
      controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
      controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    protected:
      // parameters
      std::string move_group_;
      std::string tip_link_;
      std::vector<std::string> joint_names_;
      int num_joints_;

      

      // panda setup
      rclcpp::Client<franka_msgs::srv::SetForceTorqueCollisionBehavior>::SharedPtr panda_client_;

      // MoveIt variables
      robot_model_loader::RobotModelLoaderPtr model_loader_;
      moveit::core::RobotModelPtr robot_model_;
      moveit::core::RobotStatePtr robot_state_;
      const moveit::core::JointModelGroup* joint_model_group_;
      const moveit::core::LinkModel* tip_link_model_;
      // Pre-allocated matrices and vectors for real-time loop
      Eigen::MatrixXd jacobian_;
      std::vector<double> current_joint_positions_;

      rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscriber_;
      realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::WrenchStamped>> rt_command_ptr_;
  };
}  // namespace space_panda_controller

#endif  // SPACE_PANDA_CONTROLLER__SPACE_PANDA_CONTROLLER_HPP_
