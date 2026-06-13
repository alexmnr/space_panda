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
      }
      ready_ = true;
      RCLCPP_INFO(this->get_logger(), "Ready!");
    } else {
      ready_ = false;
    }
  }

  // --- Main Mimicing Loop
  void SpacePandaLink::loop_callback() {
    if (ready_) {
      Wrench command_wrench = get_zero_wrench();
      // Mimicing
      if (mimicing_enabled_) {
        // Get current leader target
        auto target_tf_opt = get_current_tf("leader_reference", leader_tf_prefix_+"ft_frame");
        if (!target_tf_opt.has_value()) {return;}
        Transform target_tf = target_tf_opt.value();
        // Create Pose Stamped Message
        PoseStamped msg = PoseStamped();
        msg.header.frame_id = "follower_reference";
        msg.header.stamp = this->get_clock()->now();
        msg.pose = create_scaled_pose(target_tf, mimicing_scale_);
        // Publish Pose
        follower_pose_publisher_->publish(msg);
      }
      // Wrench passthrough
      if (wrench_passthrough_enabled_) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          command_wrench.force.x  += wrench_force_scale_  * filtered_input_wrench_.force.x;
          command_wrench.force.y  += wrench_force_scale_  * filtered_input_wrench_.force.y;
          command_wrench.force.z  += wrench_force_scale_  * filtered_input_wrench_.force.z;
          command_wrench.torque.x += wrench_torque_scale_ * filtered_input_wrench_.torque.x;
          command_wrench.torque.y += wrench_torque_scale_ * filtered_input_wrench_.torque.y;
          command_wrench.torque.z += wrench_torque_scale_ * filtered_input_wrench_.torque.z;
        }
      }
      // Damping
      if (damping_enabled_) {
        // Get current transform
        auto current_leader_tf_opt = get_current_tf(leader_tf_prefix_+"base_link", leader_tf_prefix_+"ft_frame");
        if (current_leader_tf_opt.has_value()) {
          current_leader_tf_ = current_leader_tf_opt.value();
          current_tf_time_ = this->get_clock()->now();
          // Calculate velocity
          if (is_first_damping_run_) {
            // Update previous states
            previous_leader_tf_ = current_leader_tf_;
            previous_tf_time_ = current_tf_time_;
            is_first_damping_run_ = false;
          } else {
            // Calculate linear velocity
            tf2::Vector3 linear_velocity = calculate_linear_velocity(current_leader_tf_, previous_leader_tf_, current_tf_time_, previous_tf_time_);
            // Check if force threshold is reached
            double absolute_force = 0.0;
            {
              std::lock_guard<std::mutex> lock(data_mutex_);
              absolute_force = abs(adjusted_input_wrench_.force.x) + abs(adjusted_input_wrench_.force.y) + abs(adjusted_input_wrench_.force.z);
            }
            if (absolute_force >= damping_force_threshold_ || !wrench_passthrough_enabled_) {
              // Calculate damping force
              command_wrench.force.x -= damping_value_ * linear_velocity.x();
              command_wrench.force.y -= damping_value_ * linear_velocity.y();
              command_wrench.force.z -= damping_value_ * linear_velocity.z();
            }
            // Update previous states
            previous_leader_tf_ = current_leader_tf_;
            previous_tf_time_ = current_tf_time_;
          }
        }
      }
      // Create final command wrench message
      WrenchStamped msg;
      msg.header.stamp = this->get_clock()->now();
      msg.wrench = command_wrench;
      leader_wrench_publisher_->publish(msg);
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
