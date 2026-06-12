#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  // --- Init
  SpacePandaLink::SpacePandaLink() : Node("space_panda_link") {
    // --- Parameters
    setup_parameters();

    // --- Initialize Services, Publishers and Subscribers
    setup_ros_interfaces();

    // --- Control Loop (500Hz)
    loop_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1), std::bind(&SpacePandaLink::loop_callback, this) ,
        loop_callback_group_
        );

    RCLCPP_INFO(this->get_logger(), "Starting Space Panda Link.");
  }

  // --- Setup
  void SpacePandaLink::setup() {
    if (enabled_) {
      if (wrench_passthrough_enabled_) {
        // Calibrating wrench data
        calibrate_input_wrench();
        RCLCPP_INFO(this->get_logger(), "Starting wrench passthrough!");
      }
      if (mimicing_enabled_) {
        // Enable Pose Servo
        set_servo_command_type(moveit_msgs::srv::ServoCommandType::Request::POSE);
        // Setting Reference
        set_reference();
        RCLCPP_INFO(this->get_logger(), "Starting mimicing!");
      }
      ready_ = true;
    } else {
      ready_ = false;
    }
  }

  // --- Main Mimicing Loop
  void SpacePandaLink::loop_callback() {
    if (ready_) {
      if (mimicing_enabled_) {
        // Get current leader target
        auto target_tf_opt = get_current_tf("leader_reference", leader_tf_prefix_+"ft_frame");
        if (!target_tf_opt.has_value()) {return;}
        Transform target_tf = target_tf_opt.value();
        // Create Pose Stamped Message
        PoseStamped msg = PoseStamped();
        msg.header.frame_id = "follower_reference";
        msg.header.stamp = this->get_clock()->now();
        msg.pose = create_scaled_pose(target_tf, mimic_scale_);
        // Publish Pose
        follower_pose_publisher_->publish(msg);
      }
      if (wrench_passthrough_enabled_) {
        WrenchStamped msg;
        msg.header.stamp = this->get_clock()->now();
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          msg.wrench = filtered_input_wrench_;
        }
        leader_wrench_publisher_->publish(msg);
      }
    }
  }

  // --- Shutdown
  void SpacePandaLink::shutdown() {
    // Stop loop
    ready_ = false;
    // Publish Zero Wrench
    publish_zero_wrench();
    RCLCPP_INFO(this->get_logger(), "Successfully shut down!");
  }
}  // namespace space_panda_link

int main(int argc, char** argv) {
  // Start Node in Thread
  rclcpp::init(argc, argv);
  auto node = std::make_shared<space_panda_link::SpacePandaLink>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros_thread([&executor]() {
      executor.spin();
      });

  // Initial Setup
  node->setup();

  // Wait for CTRL-C
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Starting Shutting down
  RCLCPP_INFO(node->get_logger(), "Ctrl-C detected! Shutting down...");
  node->shutdown();

  // Final Shutdown Sequence
  executor.cancel(); 
  if (ros_thread.joinable()) {
    ros_thread.join();
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
