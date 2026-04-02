// Copyright 2026 selfpatch
// Licensed under the Apache License, Version 2.0

/// @file joint_driver.cpp
/// @brief Joint driver node for Actuation ECU
///
/// Subscribes to motor_status (JointState), passes through velocities,
/// accumulates position from velocity * dt, and publishes joint_state.
/// Supports overheat injection (ERROR diagnostic) and position drift.

#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace actuation
{

class JointDriverNode : public rclcpp::Node
{
public:
  JointDriverNode()
  : Node("joint_driver"),
    rng_(std::random_device{}()),
    uniform_dist_(0.0, 1.0),
    last_update_time_(this->now())
  {
    // Declare parameters
    this->declare_parameter("inject_overheat", false);
    this->declare_parameter("drift_rate", 0.0);
    this->declare_parameter("failure_probability", 0.0);
    this->declare_parameter("driver_rate", 50.0);  // Hz

    load_parameters();

    // Create publishers
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Subscribe to motor_status (relative topic - within /actuation namespace)
    motor_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "motor_status", 10,
      std::bind(&JointDriverNode::on_motor_status, this, std::placeholders::_1));

    // Create timer (with rate validation)
    double rate = driver_rate_;
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'driver_rate' must be positive; using default 50.0 Hz instead of %.3f",
        rate);
      rate = 50.0;
      driver_rate_ = rate;
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&JointDriverNode::publish_joint_state, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&JointDriverNode::on_parameter_change, this, std::placeholders::_1));

    // Initialize position tracking for 2 joints (left_wheel, right_wheel)
    accumulated_position_ = {0.0, 0.0};
    latest_velocity_ = {0.0, 0.0};
    latest_effort_ = {0.0, 0.0};

    RCLCPP_INFO(this->get_logger(), "Joint driver started at %.1f Hz", rate);
  }

private:
  void load_parameters()
  {
    inject_overheat_ = this->get_parameter("inject_overheat").as_bool();
    drift_rate_ = this->get_parameter("drift_rate").as_double();
    failure_probability_ = this->get_parameter("failure_probability").as_double();
    driver_rate_ = this->get_parameter("driver_rate").as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "inject_overheat") {
        inject_overheat_ = param.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Overheat injection %s",
          inject_overheat_ ? "enabled" : "disabled");
      } else if (param.get_name() == "drift_rate") {
        drift_rate_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Drift rate changed to %.4f", drift_rate_);
      } else if (param.get_name() == "failure_probability") {
        failure_probability_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Failure probability changed to %.2f",
          failure_probability_);
      } else if (param.get_name() == "driver_rate") {
        double rate = param.as_double();
        if (rate <= 0.0) {
          result.successful = false;
          result.reason = "driver_rate must be positive";
          return result;
        }
        driver_rate_ = rate;
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          std::bind(&JointDriverNode::publish_joint_state, this));
        RCLCPP_INFO(this->get_logger(), "Driver rate changed to %.1f Hz", driver_rate_);
      }
    }

    return result;
  }

  void on_motor_status(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (msg->velocity.size() >= 2) {
      latest_velocity_ = {msg->velocity[0], msg->velocity[1]};
    }
    if (msg->effort.size() >= 2) {
      latest_effort_ = {msg->effort[0], msg->effort[1]};
    }
    has_motor_data_ = true;
  }

  void publish_joint_state()
  {
    msg_count_++;

    // Check for complete failure
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_diagnostics("DRIVER_FAILURE", "Joint driver failure (injected)");
      return;
    }

    auto now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    // Clamp dt to avoid huge jumps on first tick or after pause
    if (dt > 1.0) {
      dt = 1.0;
    }

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now;
    msg.name = {"left_wheel", "right_wheel"};

    // Get latest motor data
    std::vector<double> velocity;
    std::vector<double> effort;
    {
      std::lock_guard<std::mutex> lock(motor_mutex_);
      velocity = latest_velocity_;
      effort = latest_effort_;
    }

    // Pass through velocities
    msg.velocity = velocity;
    msg.effort = effort;

    // Accumulate position from velocity * dt + drift
    accumulated_drift_ += drift_rate_ * dt;
    for (size_t i = 0; i < accumulated_position_.size(); ++i) {
      accumulated_position_[i] += velocity[i] * dt + drift_rate_ * dt;
    }
    msg.position = accumulated_position_;

    joint_pub_->publish(msg);

    // Diagnostics
    if (inject_overheat_) {
      publish_diagnostics("OVERHEAT", "Joint driver overheat warning - temperature critical");
    } else if (std::abs(accumulated_drift_) > 0.1) {
      publish_diagnostics(
        "DRIFTING",
        "Position drift: " + std::to_string(accumulated_drift_) + " rad");
    } else if (!has_motor_data_) {
      publish_diagnostics("NO_MOTOR_DATA", "No motor status received yet");
    } else {
      publish_diagnostics("OK", "Operating normally");
    }
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "joint_driver";
    diag.hardware_id = "actuation_joints";

    if (status == "OK") {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (status == "NO_MOTOR_DATA") {
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

    kv.key = "accumulated_drift";
    kv.value = std::to_string(accumulated_drift_);
    diag.values.push_back(kv);

    kv.key = "inject_overheat";
    kv.value = inject_overheat_ ? "true" : "false";
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Motor data (protected by mutex for thread safety)
  std::mutex motor_mutex_;
  std::vector<double> latest_velocity_;
  std::vector<double> latest_effort_;
  bool has_motor_data_{false};

  // Position tracking
  std::vector<double> accumulated_position_;
  double accumulated_drift_{0.0};
  rclcpp::Time last_update_time_;

  // Parameters
  bool inject_overheat_;
  double drift_rate_;
  double failure_probability_;
  double driver_rate_;

  // Statistics
  uint64_t msg_count_{0};
};

}  // namespace actuation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<actuation::JointDriverNode>());
  rclcpp::shutdown();
  return 0;
}
