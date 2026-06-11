#include "space_panda_link/space_panda_link.hpp"

namespace space_panda_link
{
  // --- Setup: Setup Parameters 
  void SpacePandaLink::setup_parameters() {
    // Top-level parameters
    this->declare_parameter("enabled", true);
    enabled_ = this->get_parameter("enabled").as_bool();
    this->declare_parameter("leader_ns", "panda");
    leader_ns_ = this->get_parameter("leader_ns").as_string();
    leader_tf_prefix_ = "";
    if (leader_ns_ != "") {
      leader_tf_prefix_ = leader_ns_ + "_";
      leader_ns_ = "/" + leader_ns_;
    }
    this->declare_parameter("follower_ns", "ur20");
    follower_ns_ = this->get_parameter("follower_ns").as_string();
    follower_tf_prefix_ = "";
    if (follower_ns_ != "") {
      follower_tf_prefix_ = follower_ns_ + "_";
      follower_ns_ = "/" + follower_ns_;
    }

    // Wrench passthrough parameters
    this->declare_parameter("wrench_passthrough.enabled", true);
    wrench_passthrough_enabled_ = this->get_parameter("wrench_passthrough.enabled").as_bool();

      // Force parameters
    this->declare_parameter("wrench_passthrough.force.transient_limit", 10.0);
    force_transient_limit_ = this->get_parameter("wrench_passthrough.force.transient_limit").as_double();
    this->declare_parameter("wrench_passthrough.force.transient_scale", 1.0);
    force_transient_scale_ = this->get_parameter("wrench_passthrough.force.transient_scale").as_double();
    this->declare_parameter("wrench_passthrough.force.steady_scale", 0.5);
    force_steady_scale_ = this->get_parameter("wrench_passthrough.force.steady_scale").as_double();
    this->declare_parameter("wrench_passthrough.force.steady_alpha", 0.01);
    force_steady_alpha_ = this->get_parameter("wrench_passthrough.force.steady_alpha").as_double();

    // Torque parameters
    this->declare_parameter("wrench_passthrough.torque.transient_limit", 10.0);
    torque_transient_limit_ = this->get_parameter("wrench_passthrough.torque.transient_limit").as_double();
    this->declare_parameter("wrench_passthrough.torque.transient_scale", 1.0);
    torque_transient_scale_ = this->get_parameter("wrench_passthrough.torque.transient_scale").as_double();
    this->declare_parameter("wrench_passthrough.torque.steady_scale", 0.5);
    torque_steady_scale_ = this->get_parameter("wrench_passthrough.torque.steady_scale").as_double();
    this->declare_parameter("wrench_passthrough.torque.steady_alpha", 0.01);
    torque_steady_alpha_ = this->get_parameter("wrench_passthrough.torque.steady_alpha").as_double();

    // Mimicing parameters
    this->declare_parameter("mimicing.enabled", true);
    mimicing_enabled_ = this->get_parameter("mimicing.enabled").as_bool();
    this->declare_parameter("mimicing.scale", 1.0);
    mimic_scale_ = this->get_parameter("mimicing.scale").as_double();

    // Parameter Change Callback
    param_callback = this->add_on_set_parameters_callback(
            std::bind(&SpacePandaLink::on_parameter_change, this, std::placeholders::_1)
        );
  }

  // --- Setup: Setup ROS Interfaces 
  void SpacePandaLink::setup_ros_interfaces() {
    // Callback Groups
    interface_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = interface_callback_group_;
    rclcpp::QoS services_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default));

    // Follower Wrench Subscription
    follower_wrench_subscriber_ = this->create_subscription<WrenchStamped>(
        follower_ns_ + "/wrench",
        // "/fake_wrench",
        rclcpp::SensorDataQoS(),
        std::bind(&SpacePandaLink::follower_wrench_callback, this, std::placeholders::_1),
        sub_options
        );
    // Leader Wrench Publisher
    leader_wrench_publisher_ = this->create_publisher<WrenchStamped>(
        leader_ns_ + "/space_panda_controller/command_wrench", 10
        );
    // Follower Servo Client
    follower_servo_client_ = this->create_client<ServoCommandType>(
        follower_ns_ + "/servo_node/switch_command_type",
        services_qos,
        interface_callback_group_
        );
    // Leader Pose Publisher
    follower_pose_publisher_ = this->create_publisher<PoseStamped>(
        follower_ns_ + "/servo_node/pose_target_cmds", 10
        );
    // TF
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_dynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // --- Setup: On Parameter Change Callback
  rcl_interfaces::msg::SetParametersResult SpacePandaLink::on_parameter_change(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters) {
      const std::string &name = param.get_name();

      if (name == "enabled") {
        enabled_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: enabled -> %s", 
            enabled_ ? "true" : "false");
        setup();
      } 
      else if (name == "wrench_passthrough.enabled") {
        wrench_passthrough_enabled_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: wrench_passthrough.enabled -> %s", 
            wrench_passthrough_enabled_ ? "true" : "false");
      } 
      // Force changes
      else if (name == "wrench_passthrough.force.transient_limit") {
        force_transient_limit_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: wrench_passthrough.force.transient_limit -> %.2f", force_transient_limit_);
      } 
      else if (name == "wrench_passthrough.force.transient_scale") {
        force_transient_scale_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: wrench_passthrough.force.transient_scale -> %.2f", force_transient_scale_);
      } 
      else if (name == "wrench_passthrough.force.steady_scale") {
        force_steady_scale_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: wrench_passthrough.force.steady_scale -> %.2f", force_steady_scale_);
      } 
      else if (name == "wrench_passthrough.force.steady_alpha") {
        force_steady_alpha_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: wrench_passthrough.force.steady_alpha -> %.2f", force_steady_alpha_);
      } 
      // Torque changes
      else if (name == "wrench_passthrough.torque.transient_limit") {
        torque_transient_limit_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: wrench_passthrough.torque.transient_limit -> %.2f", torque_transient_limit_);
      } 
      else if (name == "wrench_passthrough.torque.transient_scale") {
        torque_transient_scale_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: wrench_passthrough.torque.transient_scale -> %.2f", torque_transient_scale_);
      } 
      else if (name == "wrench_passthrough.torque.steady_scale") {
        torque_steady_scale_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: wrench_passthrough.torque.steady_scale -> %.2f", torque_steady_scale_);
      } 
      else if (name == "wrench_passthrough.torque.steady_alpha") {
        torque_steady_alpha_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: wrench_passthrough.torque.steady_alpha -> %.2f", torque_steady_alpha_);
      } 
      // Mimicing changes
      else if (name == "mimicing.enabled") {
        mimicing_enabled_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: mimicing.enabled -> %s", 
            mimicing_enabled_ ? "true" : "false");
      } 
      else if (name == "mimicing.scale") {
        mimic_scale_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Parameter: mimicing.scale -> %.2f", mimic_scale_);
      }
    }

    return result;
  }

}  // namespace space_panda_link
