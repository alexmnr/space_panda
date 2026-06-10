#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  // --- Follower Wrench Callback
  void SpacePandaLink::follower_wrench_callback(const WrenchStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!first_wrench_received_) {
      // Initialize the filter state to the first reading
      follower_wrench_ = msg->wrench;
      first_wrench_received_ = true;
    } else {
      // Apply Exponential Moving Average (EMA) Low Pass Filter
      follower_wrench_.force.x = (wrench_filter_alpha_ * msg->wrench.force.x) + ((1.0 - wrench_filter_alpha_) * previous_follower_wrench_.force.x);
      follower_wrench_.force.y = (wrench_filter_alpha_ * msg->wrench.force.y) + ((1.0 - wrench_filter_alpha_) * previous_follower_wrench_.force.y);
      follower_wrench_.force.z = (wrench_filter_alpha_ * msg->wrench.force.z) + ((1.0 - wrench_filter_alpha_) * previous_follower_wrench_.force.z);

      follower_wrench_.torque.x = (wrench_filter_alpha_ * msg->wrench.torque.x) + ((1.0 - wrench_filter_alpha_) * previous_follower_wrench_.torque.x);
      follower_wrench_.torque.y = (wrench_filter_alpha_ * msg->wrench.torque.y) + ((1.0 - wrench_filter_alpha_) * previous_follower_wrench_.torque.y);
      follower_wrench_.torque.z = (wrench_filter_alpha_ * msg->wrench.torque.z) + ((1.0 - wrench_filter_alpha_) * previous_follower_wrench_.torque.z);
    }
    previous_follower_wrench_ = follower_wrench_;
  }

  // --- Calculate Command Wrench
  SpacePandaLink::Wrench SpacePandaLink::calculate_command_wrench(Wrench input_wrench) {
    Wrench command_wrench;
    command_wrench.force.x  = force_p_gain_ *  apply_deadband( input_wrench.force.y,  force_threshold_);
    command_wrench.force.y  = force_p_gain_ *  apply_deadband(-input_wrench.force.x,  force_threshold_);
    command_wrench.force.z  = force_p_gain_ *  apply_deadband( input_wrench.force.z,  force_threshold_);
    command_wrench.torque.x = torque_p_gain_ * apply_deadband( input_wrench.torque.y, torque_threshold_);
    command_wrench.torque.y = torque_p_gain_ * apply_deadband(-input_wrench.torque.x, torque_threshold_);
    command_wrench.torque.z = torque_p_gain_ * apply_deadband( input_wrench.torque.z, torque_threshold_);
    return command_wrench;
  }

  // --- Publish Zero Wrench
  void SpacePandaLink::publish_zero_wrench() {
    WrenchStamped msg = WrenchStamped();
    msg.wrench.force.x = 0.0;
    msg.wrench.force.y = 0.0;
    msg.wrench.force.z = 0.0;
    msg.wrench.torque.x = 0.0;
    msg.wrench.torque.y = 0.0;
    msg.wrench.torque.z = 0.0;
    leader_wrench_publisher_->publish(msg);
  }

}  // namespace space_panda_link
