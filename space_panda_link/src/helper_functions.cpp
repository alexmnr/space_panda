#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  // --- Helper Function: Apply Deadband
  double SpacePandaLink::apply_deadband(double value, double threshold) {
    if (std::abs(value) <= threshold) return 0.0;
    return (value > 0) ? (value - threshold) : (value + threshold);
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
      RCLCPP_DEBUG(this->get_logger(), "Transform lookup failed between %s and %s: %s", 
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

  // --- Helper Function: Create Scaled Pose
  SpacePandaLink::Pose SpacePandaLink::create_scaled_pose(Transform tf, double scale) {
    Pose pose = Pose();
    // Scale translation
    pose.position.x = scale * tf.translation.x;
    pose.position.y = scale * tf.translation.y;
    pose.position.z = scale * tf.translation.z;
    // Scale Rotation
    if (scale > 1.0) {
      scale = 1.0;
    }
    tf2::Quaternion target_q;
    tf2::fromMsg(tf.rotation, target_q);
    target_q.normalize();
    tf2::Vector3 axis = target_q.getAxis();
    double angle = target_q.getAngle();
    tf2::Quaternion scaled_q;
    if (std::abs(angle) > 1e-5) {
      double scaled_angle = angle * scale;
      scaled_q.setRotation(axis, scaled_angle);
    } else {
      scaled_q = tf2::Quaternion::getIdentity();
    }
    pose.orientation = tf2::toMsg(scaled_q);
    return pose;
  }

  // --- Helper Function: Get Leftover
  double SpacePandaLink::get_leftover(double value, double threshold) {
    if (value > threshold) {
        return value - threshold;
    } else if (value < -threshold) {
        return value + threshold;
    }
    return 0.0;
  }
}  // namespace space_panda_link
