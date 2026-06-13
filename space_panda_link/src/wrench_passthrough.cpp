#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  // --- Follower Wrench Callback
  void SpacePandaLink::follower_wrench_callback(const WrenchStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    adjusted_input_wrench_.force.x  =  msg->wrench.force.y  - input_wrench_offset_.force.x;
    adjusted_input_wrench_.force.y  =  -msg->wrench.force.x  - input_wrench_offset_.force.y;
    adjusted_input_wrench_.force.z  =  msg->wrench.force.z  - input_wrench_offset_.force.z;
    adjusted_input_wrench_.torque.x =  msg->wrench.torque.y - input_wrench_offset_.torque.x;
    adjusted_input_wrench_.torque.y =  -msg->wrench.torque.x - input_wrench_offset_.torque.y;
    adjusted_input_wrench_.torque.z =  msg->wrench.torque.z - input_wrench_offset_.torque.z;
    filtered_input_wrench_.force.x  = (wrench_filter_alpha_ * adjusted_input_wrench_.force.x ) + ((1 - wrench_filter_alpha_) * filtered_input_wrench_.force.x );
    filtered_input_wrench_.force.y  = (wrench_filter_alpha_ * adjusted_input_wrench_.force.y ) + ((1 - wrench_filter_alpha_) * filtered_input_wrench_.force.y );
    filtered_input_wrench_.force.z  = (wrench_filter_alpha_ * adjusted_input_wrench_.force.z ) + ((1 - wrench_filter_alpha_) * filtered_input_wrench_.force.z );
    filtered_input_wrench_.torque.x = (wrench_filter_alpha_ * adjusted_input_wrench_.torque.x) + ((1 - wrench_filter_alpha_) * filtered_input_wrench_.torque.x);
    filtered_input_wrench_.torque.y = (wrench_filter_alpha_ * adjusted_input_wrench_.torque.y) + ((1 - wrench_filter_alpha_) * filtered_input_wrench_.torque.y);
    filtered_input_wrench_.torque.z = (wrench_filter_alpha_ * adjusted_input_wrench_.torque.z) + ((1 - wrench_filter_alpha_) * filtered_input_wrench_.torque.z);
    return;
  }

  // --- Publish Zero Wrench
  void SpacePandaLink::calibrate_input_wrench() {
    RCLCPP_INFO(this->get_logger(), "Calibrating follower wrench...");
    int counter = 0;
    const int calibration_length = 100;
    Wrench temp_wrench_offset;
    // Reset current offset
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      input_wrench_offset_ = get_zero_wrench();
    }
    while (rclcpp::ok()) {
      // get current wrench
      Wrench temp_wrench;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        temp_wrench = adjusted_input_wrench_;
      }
      if (temp_wrench.force.z == 0.0) {continue;} // Wait for data
                                                  // Add to offset
      temp_wrench_offset.force.x  += temp_wrench.force.x;
      temp_wrench_offset.force.y  += temp_wrench.force.y;
      temp_wrench_offset.force.z  += temp_wrench.force.z;
      temp_wrench_offset.torque.x += temp_wrench.torque.x;
      temp_wrench_offset.torque.y += temp_wrench.torque.y;
      temp_wrench_offset.torque.z += temp_wrench.torque.z;
      counter++;
      if (counter >= calibration_length) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Final offset calculations
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      input_wrench_offset_.force.x  = temp_wrench_offset.force.x  / calibration_length;
      input_wrench_offset_.force.y  = temp_wrench_offset.force.y  / calibration_length;
      input_wrench_offset_.force.z  = temp_wrench_offset.force.z  / calibration_length;
      input_wrench_offset_.torque.x = temp_wrench_offset.torque.x / calibration_length;
      input_wrench_offset_.torque.y = temp_wrench_offset.torque.y / calibration_length;
      input_wrench_offset_.torque.z = temp_wrench_offset.torque.z / calibration_length;
      filtered_input_wrench_ = get_zero_wrench();
    }
    RCLCPP_INFO(this->get_logger(), "Successfully calibrated follower wrench.");
  }

  // --- Publish Zero Wrench
  void SpacePandaLink::publish_zero_wrench() {
    WrenchStamped msg = WrenchStamped();
    msg.header.stamp = this->get_clock()->now();
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
