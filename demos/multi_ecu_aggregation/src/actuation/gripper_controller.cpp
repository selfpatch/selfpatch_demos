// Copyright 2026 selfpatch
// Licensed under the Apache License, Version 2.0

/// @file gripper_controller.cpp
/// @brief Gripper controller node for Actuation ECU
///
/// Subscribes to /planning/commands (Twist), maps linear.z to gripper
/// open/close motion. Tracks gripper position [0.0 = closed, 1.0 = open]
/// and publishes gripper_state (JointState). Supports jam fault injection.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace actuation
{

class GripperControllerNode : public rclcpp::Node
{
public:
  GripperControllerNode()
  : Node("gripper_controller"),
    rng_(std::random_device{}()),
    uniform_dist_(0.0, 1.0),
    last_update_time_(this->now())
  {
    // Declare parameters
    this->declare_parameter("inject_jam", false);
    this->declare_parameter("failure_probability", 0.0);
    this->declare_parameter("gripper_rate", 10.0);  // Hz

    load_parameters();

    // Create publishers
    gripper_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("gripper_state", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Subscribe to planning commands (absolute topic - bridged from domain 20)
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/planning/commands", 10,
      std::bind(&GripperControllerNode::on_command, this, std::placeholders::_1));

    // Create timer (with rate validation)
    double rate = gripper_rate_;
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'gripper_rate' must be positive; using default 10.0 Hz instead of %.3f",
        rate);
      rate = 10.0;
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GripperControllerNode::publish_gripper_state, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&GripperControllerNode::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Gripper controller started at %.1f Hz", rate);
  }

private:
  void load_parameters()
  {
    inject_jam_ = this->get_parameter("inject_jam").as_bool();
    failure_probability_ = this->get_parameter("failure_probability").as_double();
    gripper_rate_ = this->get_parameter("gripper_rate").as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "inject_jam") {
        inject_jam_ = param.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Jam injection %s",
          inject_jam_ ? "enabled" : "disabled");
      } else if (param.get_name() == "failure_probability") {
        failure_probability_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Failure probability changed to %.2f",
          failure_probability_);
      } else if (param.get_name() == "gripper_rate") {
        gripper_rate_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Gripper rate changed to %.1f Hz", gripper_rate_);
      }
    }

    return result;
  }

  void on_command(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_z_command_ = msg->linear.z;
    has_cmd_ = true;
  }

  void publish_gripper_state()
  {
    msg_count_++;

    // Check for complete failure
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_diagnostics("GRIPPER_FAILURE", "Gripper failure (injected)");
      return;
    }

    auto now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    // Clamp dt to avoid huge jumps on first tick or after pause
    if (dt > 1.0) {
      dt = 1.0;
    }

    // Get latest command
    double z_cmd = 0.0;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      z_cmd = latest_z_command_;
    }

    // Update gripper position (unless jammed)
    if (!inject_jam_) {
      // Positive z = open, negative z = close
      // Scale command to reasonable gripper speed
      gripper_position_ += z_cmd * dt;
      gripper_position_ = std::clamp(gripper_position_, 0.0, 1.0);
    }

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now;
    msg.name = {"gripper_finger"};
    msg.position = {gripper_position_};
    msg.velocity = {inject_jam_ ? 0.0 : z_cmd};
    msg.effort = {0.0};

    gripper_pub_->publish(msg);

    // Diagnostics
    if (inject_jam_) {
      publish_diagnostics(
        "JAMMED",
        "Gripper jammed at position " + std::to_string(gripper_position_));
    } else if (!has_cmd_) {
      publish_diagnostics("NO_COMMAND", "No command received yet");
    } else {
      publish_diagnostics("OK", "Operating normally");
    }
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "gripper_controller";
    diag.hardware_id = "actuation_gripper";

    if (status == "OK") {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (status == "NO_COMMAND") {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    diag.message = message;

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "status";
    kv.value = status;
    diag.values.push_back(kv);

    kv.key = "msg_count";
    kv.value = std::to_string(msg_count_);
    diag.values.push_back(kv);

    kv.key = "gripper_position";
    kv.value = std::to_string(gripper_position_);
    diag.values.push_back(kv);

    kv.key = "inject_jam";
    kv.value = inject_jam_ ? "true" : "false";
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Command data (protected by mutex for thread safety)
  std::mutex cmd_mutex_;
  double latest_z_command_{0.0};
  bool has_cmd_{false};

  // Gripper state
  double gripper_position_{0.0};  // 0.0 = closed, 1.0 = open
  rclcpp::Time last_update_time_;

  // Parameters
  bool inject_jam_;
  double failure_probability_;
  double gripper_rate_;

  // Statistics
  uint64_t msg_count_{0};
};

}  // namespace actuation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<actuation::GripperControllerNode>());
  rclcpp::shutdown();
  return 0;
}
