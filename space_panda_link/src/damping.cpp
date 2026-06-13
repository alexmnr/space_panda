#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  // --- Calculate linear velocity
    tf2::Vector3 SpacePandaLink::calculate_linear_velocity(Transform current_tf, Transform previous_tf, rclcpp::Time current_time, rclcpp::Time previous_time) {
      double dt = (current_time - previous_time).seconds();
      tf2::Vector3 linear_velocity(0.0, 0.0, 0.0);
      if (dt > 0.0) {
        // Get current and previous data
        tf2::Vector3 current_position, previous_position;
        tf2::fromMsg(current_tf.translation, current_position);
        tf2::fromMsg(previous_tf.translation, previous_position);
        // Linear Velocity (Global Frame)
        tf2::Vector3 v_linear_global = (current_position - previous_position) / dt;
        // Transform Velocities to Local Frame (ft_frame)
        tf2::Quaternion current_rotation;
        tf2::fromMsg(current_tf.rotation, current_rotation);
        tf2::Quaternion current_rotation_inverse = current_rotation.inverse();
        linear_velocity = tf2::quatRotate(current_rotation_inverse, v_linear_global);
      } 
      return linear_velocity;
    }

}  // namespace space_panda_link
