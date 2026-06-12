#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  // --- Set Reference
  void SpacePandaLink::set_reference() {
    RCLCPP_INFO(this->get_logger(), "Setting reference...");
    while (rclcpp::ok()) {
      // Try to get current tf of ft_frame for both follower and leader
      auto leader_tf_opt = get_current_tf("world", leader_tf_prefix_+"ft_frame");
      auto follower_tf_opt = get_current_tf("world", follower_tf_prefix_+"tool0");
      if (leader_tf_opt.has_value() && follower_tf_opt.has_value()) {
        leader_reference_ = leader_tf_opt.value();
        follower_reference_ = follower_tf_opt.value();
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    // Publish static tf references
    publish_static_tf("world", "leader_reference", leader_reference_);
    publish_static_tf("world", "follower_reference", follower_reference_);
    RCLCPP_INFO(this->get_logger(), "Successfully set reference.");
  }

  // --- Set Servo Command Type
  void SpacePandaLink::set_servo_command_type(int8_t command_type) {
    RCLCPP_INFO(this->get_logger(), "Switching Servo Command Type...");
    while (rclcpp::ok()) {
      // Wait for the service to be available (returns false if it times out)
      if (!follower_servo_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Service not available after timeout.");
        continue;
      }
      auto request = std::make_shared<ServoCommandType::Request>();
      request->command_type = command_type;
      auto future_result = follower_servo_client_->async_send_request(request);
      // Block current thread until the response is ready
      try {
        auto response = future_result.get();
        if (response && response->success) {
          RCLCPP_INFO(this->get_logger(), "Successfully switched Servo Command Type.");
          return;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to update servo command type.");
          continue;
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed with exception: %s", e.what());
        continue;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

}  // namespace space_panda_link
