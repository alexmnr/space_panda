#ifndef SPACE_PANDA_LINK__SPACE_PANDA_LINK_HPP_
#define SPACE_PANDA_LINK__SPACE_PANDA_LINK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace space_panda_link
{
  class SpacePandaLink : public rclcpp::Node {
    public:
      using WrenchStamped = geometry_msgs::msg::WrenchStamped;
      using Wrench = geometry_msgs::msg::Wrench;
      using Pose = geometry_msgs::msg::Pose;
      using PoseStamped = geometry_msgs::msg::PoseStamped;
      using TwistStamped = geometry_msgs::msg::TwistStamped;
      using Transform = geometry_msgs::msg::Transform;
      using ServoCommandType = moveit_msgs::srv::ServoCommandType;
      void setup();
      void shutdown();
      SpacePandaLink();

    private:
      // Parameters
      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback;
      rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);
      std::string leader_ns_;
      std::string leader_tf_prefix_;
      std::string follower_ns_;
      std::string follower_tf_prefix_;
      double force_transient_limit_;
      double force_transient_scale_;
      double force_steady_scale_;
      double force_steady_alpha_;
      double torque_transient_limit_;
      double torque_transient_scale_;
      double torque_steady_scale_;
      double torque_steady_alpha_;
      double mimic_scale_;
      bool wrench_passthrough_enabled_;
      bool mimicing_enabled_;
      bool enabled_;

      // Setup
      rclcpp::CallbackGroup::SharedPtr interface_callback_group_;
      rclcpp::CallbackGroup::SharedPtr loop_callback_group_;
      void setup_parameters();
      void setup_ros_interfaces();

      // State
      // std::mutex data_mutex_;
      bool ready_;

      // Main Loop
      rclcpp::TimerBase::SharedPtr timer_;
      void loop_callback();

      // Wrench Passthrough
      rclcpp::Subscription<WrenchStamped>::SharedPtr follower_wrench_subscriber_;
      rclcpp::Publisher<WrenchStamped>::SharedPtr leader_wrench_publisher_;
      void follower_wrench_callback(const WrenchStamped::SharedPtr msg);
      // Wrench input_wrench_;
      // Wrench d_input_wrench;
      bool first_wrench_received_ = false;
      WrenchStamped current_input_wrench_msg_;
      WrenchStamped previous_input_wrench_msg_;
      Wrench transient_command_wrench_;
      // Wrench calculate_command_wrench();

      // Mimicing
      rclcpp::Publisher<PoseStamped>::SharedPtr follower_pose_publisher_;
      rclcpp::Publisher<TwistStamped>::SharedPtr follower_twist_publisher_;
      rclcpp::Client<ServoCommandType>::SharedPtr follower_servo_client_;
      bool set_reference();
      Transform leader_reference_;
      Transform follower_reference_;
      bool set_servo_command_type(int8_t command_type);

      // Helper Functions
      double apply_deadband(double value, double threshold);
      Pose create_scaled_pose(Transform tf, double scale);
      void publish_zero_wrench();
      Wrench get_zero_wrench();
      double get_leftover(double value, double threshold);

      // spring test
      double last_z_ = 0.0;
      std::chrono::steady_clock::time_point last_time_;
      bool is_first_run_ = true;

      // TF 
      std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_dynamic_broadcaster_;
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
      void publish_static_tf(const std::string & frame_id, const std::string & child_id, const Transform & tf);
      void publish_dynamic_tf(const std::string & frame_id, const std::string & child_id, const Transform & tf);
      std::optional<geometry_msgs::msg::Transform> get_current_tf(const std::string & frame_id, const std::string & child_id);
      void print_tf(const std::string & label, const Transform & tf);
  };
}  // namespace space_panda_link

#endif  // SPACE_PANDA_LINK__SPACE_PANDA_LINK_HPP_
