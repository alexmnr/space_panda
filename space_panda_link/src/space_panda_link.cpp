#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  SpacePandaLink::SpacePandaLink() : Node("space_panda_link") {
    // Parameters
    this->declare_parameter("leader_ns", "panda");
    this->declare_parameter("follower_ns", "ur20");
    this->declare_parameter("wrench_scale", 1.0);
    this->declare_parameter("mimic_scale", 1.0);
    this->declare_parameter("wrench_passthrough", true);
    wrench_scale_ = this->get_parameter("wrench_scale").as_double();
    mimic_scale_ = this->get_parameter("mimic_scale").as_double();
    leader_ns_ = this->get_parameter("leader_ns").as_string();
    wrench_passthrough_ = this->get_parameter("wrench_passthrough").as_bool();
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
        follower_ns_ + "/servo_node/pose_target_cmds", 10
        );

    // TF
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_dynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Control Loop (100Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&SpacePandaLink::loop_callback, this) 
        );

    RCLCPP_INFO(this->get_logger(), "Starting Space Panda Link!");
  }

  // --- Main Loop
  void SpacePandaLink::loop_callback() {
    // Set Reference
    if (mode == Mode::SetReference) {
      RCLCPP_INFO(this->get_logger(), "Trying to set Reference...");
      // Try to get current tf of ft_frame for both follower and leader
      auto leader_tf_opt = get_current_tf("world", leader_tf_prefix_+"ft_frame");
      if (!leader_tf_opt.has_value()) {return;}
      leader_reference_ = leader_tf_opt.value();
      auto follower_tf_opt = get_current_tf("world", follower_tf_prefix_+"tool0");
      if (!follower_tf_opt.has_value()) {return;}
      follower_reference_ = follower_tf_opt.value();
      // Publish static tf references
      publish_static_tf("world", "leader_reference", leader_reference_);
      publish_static_tf("world", "follower_reference", follower_reference_);
      RCLCPP_INFO(this->get_logger(), "Successfully set reference.");
      mode = Mode::CalibrateWrench;
    // Calibrate Wrench
    } else if (mode == Mode::CalibrateWrench) {
      RCLCPP_INFO(this->get_logger(), "Trying to calibrate Follower Wrench...");
      RCLCPP_INFO(this->get_logger(), "Successfully calibrated Follower Wrench.");
      mode = Mode::Mimic;
      RCLCPP_INFO(this->get_logger(), "Starting Mimicing...");
    }
    // Mimic
    if (mode == Mode::Mimic) {
      // Get TF
      auto target_tf_opt = get_current_tf("leader_reference", leader_tf_prefix_+"ft_frame");
      if (!target_tf_opt.has_value()) {return;}
      Transform target_tf = target_tf_opt.value();
      // Send Pose Servo
      PoseStamped msg = PoseStamped();
      msg.header.frame_id = "follower_reference";
      msg.header.stamp = this->get_clock()->now();
      msg.pose.position.x = mimic_scale_ * target_tf.translation.x;
      msg.pose.position.y = mimic_scale_ * target_tf.translation.y;
      msg.pose.position.z = mimic_scale_ * target_tf.translation.z;
      msg.pose.orientation = target_tf.rotation;
      follower_pose_publisher_->publish(msg);
      // Apply Wrench to Leader
      if (wrench_passthrough_) {
        WrenchStamped leader_wrench_msg = WrenchStamped();
        leader_wrench_msg.header.stamp = this->get_clock()->now();
        leader_wrench_msg.wrench.force.x  = wrench_scale_ * apply_deadband(follower_wrench_.force.x , threshold_);
        leader_wrench_msg.wrench.force.y  = wrench_scale_ * apply_deadband(follower_wrench_.force.y , threshold_);
        leader_wrench_msg.wrench.force.z  = wrench_scale_ * apply_deadband(follower_wrench_.force.z , threshold_);
        leader_wrench_msg.wrench.torque.x = wrench_scale_ * apply_deadband(follower_wrench_.torque.x, threshold_);
        leader_wrench_msg.wrench.torque.y = wrench_scale_ * apply_deadband(follower_wrench_.torque.y, threshold_);
        leader_wrench_msg.wrench.torque.z = wrench_scale_ * apply_deadband(follower_wrench_.torque.z, threshold_);
        leader_wrench_publisher_->publish(leader_wrench_msg);
      }
    }

  }

  // --- Follower Wrench Callback
  void SpacePandaLink::follower_wrench_callback(const WrenchStamped::SharedPtr msg) {
    follower_wrench_ = msg->wrench;
  }

  // --- Helper Function: Apply Deadband
  double SpacePandaLink::apply_deadband(double value, double threshold) {
    if (std::abs(value) <= threshold) return 0.0;
    return (value > 0) ? (value - threshold) : (value + threshold);
  }

  // --- Helper Function: Enable Servo
  void SpacePandaLink::enable_servo() {
    RCLCPP_INFO(this->get_logger(), "Enabling Pose Servo...");
    while (!follower_servo_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for servo service...");
    }

    auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;

    auto result_future = follower_servo_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Pose Servo successfully enabled.");
  }

  // --- Helper Function: Publish Static Transform 
  void SpacePandaLink::publish_static_tf(const std::string & frame_id, const std::string & child_id, const Transform & tf) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_id;
    msg.transform = tf;
    tf_static_broadcaster_->sendTransform(msg);
  }

  // --- Helper Function: Publish Dynamic Transform 
  void SpacePandaLink::publish_dynamic_tf(const std::string & frame_id, const std::string & child_id, const Transform & tf) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_id;
    msg.transform = tf;
    tf_dynamic_broadcaster_->sendTransform(msg);
  }

  // --- Helper Function: Get Current Transform 
  std::optional<geometry_msgs::msg::Transform> SpacePandaLink::get_current_tf(const std::string & frame_id, const std::string & child_id) {
    try {
      geometry_msgs::msg::TransformStamped stamped_tf = tf_buffer_->lookupTransform(
          frame_id, 
          child_id, 
          tf2::TimePointZero
          );
      return stamped_tf.transform;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Transform lookup failed between %s and %s: %s", 
          frame_id.c_str(), child_id.c_str(), ex.what());
      return std::nullopt;
    }
  }

  // --- Helper Function: Print Transform
  void SpacePandaLink::print_tf(const std::string & label, const Transform & tf) {
    tf2::Quaternion q(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    RCLCPP_INFO(this->get_logger(), 
        "[%s] XYZ: [%.3f, %.3f, %.3f] | RPY (deg): [%.1f, %.1f, %.1f]",
        label.c_str(), 
        tf.translation.x, tf.translation.y, tf.translation.z,
        r * (180.0 / M_PI), p * (180.0 / M_PI), y * (180.0 / M_PI)
        );
  }

}  // namespace space_panda_link

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<space_panda_link::SpacePandaLink>();

  node->enable_servo();

  // Multi-threaded executor to run callbacks in parallel
  // rclcpp::executors::MultiThreadedExecutor executor;

  // executor.add_node(node);
  // executor.spin();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
