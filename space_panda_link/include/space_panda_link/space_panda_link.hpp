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
      // General parameters
      bool enabled_;
      std::string leader_ns_;
      std::string leader_tf_prefix_;
      std::string follower_ns_;
      std::string follower_tf_prefix_;
      // Damping
      double damping_enabled_;
      double damping_value_;
      double damping_force_threshold_;
      // Wrench passthrough parameters
      bool wrench_passthrough_enabled_;
      double wrench_filter_alpha_;
      double wrench_force_scale_;
      double wrench_torque_scale_;
      // Mimicing parameters
      bool mimicing_enabled_;
      double mimicing_scale_;

      // Setup
      rclcpp::CallbackGroup::SharedPtr interface_callback_group_;
      rclcpp::CallbackGroup::SharedPtr loop_callback_group_;
      void setup_parameters();
      void setup_ros_interfaces();

      // State
      std::mutex data_mutex_;
      bool ready_;

      // Main Loop
      rclcpp::TimerBase::SharedPtr timer_;
      void loop_callback();

      // Wrench Passthrough
      rclcpp::Subscription<WrenchStamped>::SharedPtr follower_wrench_subscriber_;
      rclcpp::Publisher<WrenchStamped>::SharedPtr leader_wrench_publisher_;
      void follower_wrench_callback(const WrenchStamped::SharedPtr msg);
      void calibrate_input_wrench();
      Wrench adjusted_input_wrench_;
      Wrench filtered_input_wrench_;
      Wrench input_wrench_offset_;

      // Mimicing
      rclcpp::Publisher<PoseStamped>::SharedPtr follower_pose_publisher_;
      rclcpp::Publisher<TwistStamped>::SharedPtr follower_twist_publisher_;
      rclcpp::Client<ServoCommandType>::SharedPtr follower_servo_client_;
      void set_reference();
      Transform leader_reference_;
      Transform follower_reference_;
      void set_servo_command_type(int8_t command_type);

      // Damping
      rclcpp::Time current_tf_time_;
      rclcpp::Time previous_tf_time_;
      Transform current_leader_tf_;
      Transform previous_leader_tf_;
      bool is_first_damping_run_ = true;
      tf2::Vector3 calculate_linear_velocity(Transform current_tf, Transform previous_tf, rclcpp::Time current_time, rclcpp::Time previous_time);

      // Helper Functions
      double apply_deadband(double value, double threshold);
      Pose create_scaled_pose(Transform tf, double scale);
      void publish_zero_wrench();
      Wrench get_zero_wrench();
      double get_leftover(double value, double threshold);

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
