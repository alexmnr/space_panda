#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  // --- Mimic Function: Set Reference
  bool SpacePandaLink::set_reference() {
    // Try to get current tf of ft_frame for both follower and leader
    auto leader_tf_opt = get_current_tf("world", leader_tf_prefix_+"ft_frame");
    if (!leader_tf_opt.has_value()) {return false;}
    leader_reference_ = leader_tf_opt.value();
    auto follower_tf_opt = get_current_tf("world", follower_tf_prefix_+"tool0");
    if (!follower_tf_opt.has_value()) {return false;}
    follower_reference_ = follower_tf_opt.value();
    // Publish static tf references
    publish_static_tf("world", "leader_reference", leader_reference_);
    publish_static_tf("world", "follower_reference", follower_reference_);
    return true;
  }

  // --- Mimic Function: Set Servo Command Type
  bool SpacePandaLink::set_servo_command_type(int8_t command_type) {
    // Wait for the service to be available (returns false if it times out)
    if (!follower_servo_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available after timeout.");
      return false;
    }
    auto request = std::make_shared<ServoCommandType::Request>();
    request->command_type = command_type;
    auto future_result = follower_servo_client_->async_send_request(request);
    // Block current thread until the response is ready
    try {
      auto response = future_result.get();
      if (response && response->success) {
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to update command type.");
        return false;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed with exception: %s", e.what());
      return false;
    }
  }

}  // namespace space_panda_link
