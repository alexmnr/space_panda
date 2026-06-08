#ifndef SPACE_PANDA_LINK__SPACE_PANDA_LINK_HPP_
#define SPACE_PANDA_LINK__SPACE_PANDA_LINK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <mutex>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace space_panda_link
{
  enum Mode {
    SetReference,
    CalibrateWrench,
    Mimic,
  };
  class SpacePandaLink : public rclcpp::Node {
    public:
      using WrenchStamped = geometry_msgs::msg::WrenchStamped;
      using Wrench = geometry_msgs::msg::Wrench;
      using PoseStamped = geometry_msgs::msg::PoseStamped;
      using Transform = geometry_msgs::msg::Transform;
      SpacePandaLink();
      void enable_servo();

    private:
      // Parameters
      std::string leader_ns_;
      std::string leader_tf_prefix_;
      std::string follower_ns_;
      std::string follower_tf_prefix_;
      double wrench_scale_ = 1.0;
      double mimic_scale_ = 1.0;
      int mode = 0;
      bool wrench_passthrough_;

      // State
      std::mutex data_mutex_;
      Wrench follower_wrench_;
      Transform leader_reference_;
      Transform follower_reference_;

      // Callbacks
      void loop_callback();
      void follower_wrench_callback(const WrenchStamped::SharedPtr msg);

      // Helper Functions
      double threshold_ = 0.5;
      double apply_deadband(double value, double threshold);
      void publish_static_tf(const std::string & frame_id, const std::string & child_id, const Transform & tf);
      void publish_dynamic_tf(const std::string & frame_id, const std::string & child_id, const Transform & tf);
      std::optional<geometry_msgs::msg::Transform> get_current_tf(const std::string & frame_id, const std::string & child_id);
      void print_tf(const std::string & label, const Transform & tf);

      // ROS Communications
      rclcpp::Subscription<WrenchStamped>::SharedPtr follower_wrench_subscriber_;
      rclcpp::Publisher<WrenchStamped>::SharedPtr leader_wrench_publisher_;
      rclcpp::Publisher<PoseStamped>::SharedPtr follower_pose_publisher_;
      rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr follower_servo_client_;
      rclcpp::TimerBase::SharedPtr timer_;
      std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_dynamic_broadcaster_;
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  };
}  // namespace space_panda_link

#endif  // SPACE_PANDA_LINK__SPACE_PANDA_LINK_HPP_
