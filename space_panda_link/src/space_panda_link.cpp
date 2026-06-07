#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  SpacePandaLink::SpacePandaLink() : Node("space_panda_link") {
    // Parameters
    this->declare_parameter("leader_ns", "panda");
    this->declare_parameter("follower_ns", "ur20");
    this->declare_parameter("scale", 1.0);
    scale_ = this->get_parameter("scale").as_double();
    leader_ns_ = this->get_parameter("leader_ns").as_string();
    leader_tf_prefix_ = "";
    if (leader_ns_ != "") {
      leader_tf_prefix_ = leader_ns_ + "_";
      leader_ns_ = "/" + leader_ns_;
    }
    follower_ns_ = this->get_parameter("follower_ns").as_string();
    follower_tf_prefix_ = "";
    if (follower_ns_ != "") {
      follower_tf_prefix_ = follower_ns_ + "_";
      follower_ns_ = "/" + follower_ns_;
    }

    // Follower Wrench Subscription
    follower_wrench_subscriber_ = this->create_subscription<WrenchStamped>(
        follower_ns_ + "/wrench",
        rclcpp::SensorDataQoS(),
        std::bind(&SpacePandaLink::follower_wrench_callback, this, std::placeholders::_1)
        );

    // Leader Wrench Publisher
    leader_wrench_publisher_ = this->create_publisher<WrenchStamped>(
        leader_ns_ + "/space_panda_controller/command_wrench", 10
        );

    // Follower Servo Client
    follower_servo_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(
        follower_ns_ + "/servo_node/switch_command_type"
        );

    // Leader Pose Publisher
    follower_pose_publisher_ = this->create_publisher<PoseStamped>(
        follower_ns_ + "/servo_node/pose_target_cmnds", 10
        );

    // Control Loop (100Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&SpacePandaLink::loop_callback, this) 
        );

    RCLCPP_INFO(this->get_logger(), "Starting Space Panda Link!");
  }

  void SpacePandaLink::loop_callback() {
    // RCLCPP_INFO(this->get_logger(), "Force: X: %.3f Y: %.3f Z: %.3f", follower_wrench_.force.x, follower_wrench_.force.y, follower_wrench_.force.z);
    WrenchStamped leader_wrench_msg = WrenchStamped();
    leader_wrench_msg.header.stamp = this->get_clock()->now();
    leader_wrench_msg.wrench.force.x  = scale_ * apply_deadband(-follower_wrench_.force.x , threshold_);
    leader_wrench_msg.wrench.force.y  = scale_ * apply_deadband(follower_wrench_.force.y  , threshold_);
    leader_wrench_msg.wrench.force.z  = scale_ * apply_deadband(-follower_wrench_.force.z , threshold_);
    leader_wrench_msg.wrench.torque.x = scale_ * apply_deadband(-follower_wrench_.torque.x, threshold_);
    leader_wrench_msg.wrench.torque.y = scale_ * apply_deadband(follower_wrench_.torque.y , threshold_);
    leader_wrench_msg.wrench.torque.z = scale_ * apply_deadband(-follower_wrench_.torque.z, threshold_);
    leader_wrench_publisher_->publish(leader_wrench_msg);
  }

  void SpacePandaLink::follower_wrench_callback(const WrenchStamped::SharedPtr msg) {
    follower_wrench_ = msg->wrench;
  }

  double SpacePandaLink::apply_deadband(double value, double threshold) {
    if (std::abs(value) <= threshold) return 0.0;
    return (value > 0) ? (value - threshold) : (value + threshold);
  }

  void SpacePandaLink::enable_servo() {
    while (!follower_servo_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for servo service...");
    }

    auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;

    auto result_future = follower_servo_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Pose Servo successfully enabled.");
  }

}  // namespace space_panda_link

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<space_panda_link::SpacePandaLink>();

  RCLCPP_INFO(node->get_logger(), "Enabling Pose Servo");
  node->enable_servo();

  // Multi-threaded executor to run callbacks in parallel
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
