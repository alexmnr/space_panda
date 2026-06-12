#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  // --- Follower Wrench Callback
  void SpacePandaLink::follower_wrench_callback(const WrenchStamped::SharedPtr msg) {
    return;
    // Check wheter its the first run
    if (!first_wrench_received_) {
      current_input_wrench_msg_ = *msg;
      first_wrench_received_ = true;
      return;
    } 
    // Save current input wrench message
    current_input_wrench_msg_ = *msg;
    // Calculate Command Wrench from Input Wrench
    Wrench command_wrench = get_zero_wrench();
    // Transient
    command_wrench.force.z += force_transient_scale_ * std::clamp(current_input_wrench_msg_.wrench.force.z, -force_transient_limit_, force_transient_limit_);
    // Steady
    Wrench current_leftover;
    Wrench previous_leftover;
    current_leftover.force.z = get_leftover(current_input_wrench_msg_.wrench.force.z, force_transient_limit_);
    transient_command_wrench_.force.z = force_steady_alpha_ * current_leftover.force.z + (1 - force_steady_alpha_) * transient_command_wrench_.force.z;
    command_wrench.force.z += force_steady_scale_ * transient_command_wrench_.force.z;
    // Publish command wrench
    WrenchStamped command_msg;
    command_msg.header.stamp = this->get_clock()->now();
    command_msg.wrench = command_wrench;
    if (wrench_passthrough_enabled_) {
      leader_wrench_publisher_->publish(command_msg);
    }
    // Save previous input wrench message
    previous_input_wrench_msg_ = current_input_wrench_msg_;
  }

  // --- Publish Zero Wrench
  void SpacePandaLink::publish_zero_wrench() {
    WrenchStamped msg = WrenchStamped();
    msg.wrench = get_zero_wrench();
    leader_wrench_publisher_->publish(msg);
  }

  // --- Get Zero Wrench
  SpacePandaLink::Wrench SpacePandaLink::get_zero_wrench() {
    Wrench msg = Wrench();
    msg.force.x = 0.0;
    msg.force.y = 0.0;
    msg.force.z = 0.0;
    msg.torque.x = 0.0;
    msg.torque.y = 0.0;
    msg.torque.z = 0.0;
    return msg;
  }

}  // namespace space_panda_link
