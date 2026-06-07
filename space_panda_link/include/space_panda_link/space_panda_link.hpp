#ifndef SPACE_PANDA_LINK__SPACE_PANDA_LINK_HPP_
#define SPACE_PANDA_LINK__SPACE_PANDA_LINK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <mutex>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>

namespace space_panda_link
{
  class SpacePandaLink : public rclcpp::Node {
    public:
      using WrenchStamped = geometry_msgs::msg::WrenchStamped;
      using Wrench = geometry_msgs::msg::Wrench;
      using PoseStamped = geometry_msgs::msg::PoseStamped;
      SpacePandaLink();
      void enable_servo();

    private:
      // Parameters
      std::string leader_ns_;
      std::string leader_tf_prefix_;
      std::string follower_ns_;
      std::string follower_tf_prefix_;
      double scale_;

      // State
      std::mutex data_mutex_;
      Wrench follower_wrench_;

      // Callbacks
      void loop_callback();
      void follower_wrench_callback(const WrenchStamped::SharedPtr msg);

      // Helper Functions
      double threshold_ = 0.5;
      double apply_deadband(double value, double threshold);

      // ROS Communications
      rclcpp::Subscription<WrenchStamped>::SharedPtr follower_wrench_subscriber_;
      rclcpp::Publisher<WrenchStamped>::SharedPtr leader_wrench_publisher_;
      rclcpp::Publisher<PoseStamped>::SharedPtr follower_pose_publisher_;
      rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr follower_servo_client_;
      rclcpp::TimerBase::SharedPtr timer_;
  };
}  // namespace space_panda_link

#endif  // SPACE_PANDA_LINK__SPACE_PANDA_LINK_HPP_
