#ifndef SPACE_PANDA_CONTROLLER__SPACE_PANDA_CONTROLLER_HPP_
#define SPACE_PANDA_CONTROLLER__SPACE_PANDA_CONTROLLER_HPP_

#include <string>
#include <stddef.h>
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <franka_semantic_components/franka_robot_state.hpp>

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
      std::vector<std::string> joint_names_;
      double command_force_limit_;
      double command_torque_limit_;
      bool friction_compensation_;

      // panda setup
      rclcpp::Client<franka_msgs::srv::SetForceTorqueCollisionBehavior>::SharedPtr panda_client_;
      std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
      std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

      // Friction Compensation
      std::array<double, 7> coulomb_friction_ = {0.8, 0.5, 0.3,  0.4,  0.3,  0.1,  0.2}; // [Nm]
      std::array<double, 7> viscous_friction_ = {0.1, 0.1, 0.05, 0.05, 0.05, 0.05, 0.05}; // [Nm / (rad/s)]
      double friction_smoothness_k_ = 40.0;

      rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscriber_;
      realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::WrenchStamped>> rt_command_ptr_;
  };
}  // namespace space_panda_controller

#endif  // SPACE_PANDA_CONTROLLER__SPACE_PANDA_CONTROLLER_HPP_
