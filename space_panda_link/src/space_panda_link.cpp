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
      if (mimicing_enabled_) {
        // Enable Pose Servo
        RCLCPP_INFO(this->get_logger(), "Switching Servo Command Type...");
        while (rclcpp::ok()) {
          if (set_servo_command_type(ServoCommandType::Request::POSE)) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "Successfully switched Servo Command Type.");
        // Setting Reference
        RCLCPP_INFO(this->get_logger(), "Setting reference...");
        while (rclcpp::ok()) {
          if (set_reference()) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "Successfully set reference.");
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
      // if (mimicing_enabled_) {
      //   // Get current leader target
      //   auto target_tf_opt = get_current_tf("leader_reference", leader_tf_prefix_+"ft_frame");
      //   if (!target_tf_opt.has_value()) {return;}
      //   Transform target_tf = target_tf_opt.value();
      //   // Create Pose Stamped Message
      //   PoseStamped msg = PoseStamped();
      //   msg.header.frame_id = "follower_reference";
      //   msg.header.stamp = this->get_clock()->now();
      //   msg.pose = create_scaled_pose(target_tf, mimic_scale_);
      //   // Publish Pose
      //   follower_pose_publisher_->publish(msg);
      // }
      // test spring code
      auto target_tf_opt = get_current_tf("leader_reference", leader_tf_prefix_+"ft_frame");
      if (!target_tf_opt.has_value()) {return;}
      Transform target_tf = target_tf_opt.value();

      double current_z = target_tf.translation.z;
      double delta_z = std::clamp(current_z - 0.05, 0.0, 0.1);

      // 1. Calculate time delta (dt) and Z-velocity
      double vel_z = 0.0;
      auto current_time = std::chrono::steady_clock::now();

      if (!is_first_run_) {
        std::chrono::duration<double> elapsed = current_time - last_time_;
        double dt = elapsed.count();
        if (dt > 0.0) {
          vel_z = (current_z - last_z_) / dt; // Velocity in m/s
        }
      }
      last_z_ = current_z;
      last_time_ = current_time;
      is_first_run_ = false;

      // 2. Base Spring Force calculation
      double spring_k = 10000.0; 
      double force_z = delta_z * -spring_k;

      // 3. Apply Penetration-Only Damping
      // Condition: We are inside the wall (delta_z > 0) AND moving deeper into it (vel_z > 0)
      // if (delta_z > 0.0 && vel_z > 0.0) {
      double damping_c = 050.0; // Tuning parameter (N*s/m). Start here and adjust.
      force_z += -damping_c * vel_z; // Opposes the positive velocity direction
      RCLCPP_INFO(this->get_logger(), "Wall");
      // }

      // 4. Package and output the wrench
      std::shared_ptr<WrenchStamped> msg = std::make_shared<WrenchStamped>();
      msg->wrench.force.z = force_z;
      current_input_wrench_msg_ = *msg;

      Wrench command_wrench = get_zero_wrench();
      command_wrench.force.z += force_transient_scale_ * std::clamp(current_input_wrench_msg_.wrench.force.z, -force_transient_limit_, force_transient_limit_);
      WrenchStamped command_msg;
      command_msg.header.stamp = this->get_clock()->now();
      command_msg.wrench = command_wrench;
      if (wrench_passthrough_enabled_) {
        leader_wrench_publisher_->publish(command_msg);
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
