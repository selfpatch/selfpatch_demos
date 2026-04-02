// Copyright 2026 selfpatch
// Licensed under the Apache License, Version 2.0

/// @file motor_controller.cpp
/// @brief Motor controller node for Actuation ECU
///
/// Subscribes to /planning/commands (Twist), converts to differential drive
/// wheel velocities, and publishes motor_status (JointState) with configurable
/// torque noise and failure injection.

#include <chrono>
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

class MotorControllerNode : public rclcpp::Node
{
public:
  MotorControllerNode()
  : Node("motor_controller"),
    rng_(std::random_device{}()),
    normal_dist_(0.0, 1.0),
    uniform_dist_(0.0, 1.0)
  {
    // Declare parameters
    this->declare_parameter("torque_noise", 0.01);
    this->declare_parameter("failure_probability", 0.0);
    this->declare_parameter("status_rate", 20.0);  // Hz

    load_parameters();

    // Create publishers
    motor_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("motor_status", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Subscribe to planning commands (absolute topic - bridged from domain 20)
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/planning/commands", 10,
      std::bind(&MotorControllerNode::on_command, this, std::placeholders::_1));

    // Create timer (with rate validation)
    double rate = status_rate_;
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'status_rate' must be positive; using default 20.0 Hz instead of %.3f",
        rate);
      rate = 20.0;
      status_rate_ = rate;
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MotorControllerNode::publish_motor_status, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MotorControllerNode::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Motor controller started at %.1f Hz", rate);
  }

private:
  void load_parameters()
  {
    torque_noise_ = this->get_parameter("torque_noise").as_double();
    failure_probability_ = this->get_parameter("failure_probability").as_double();
    status_rate_ = this->get_parameter("status_rate").as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "torque_noise") {
        torque_noise_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Torque noise changed to %.4f", torque_noise_);
      } else if (param.get_name() == "failure_probability") {
        failure_probability_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Failure probability changed to %.2f",
          failure_probability_);
      } else if (param.get_name() == "status_rate") {
        double rate = param.as_double();
        if (rate <= 0.0) {
          result.successful = false;
          result.reason = "status_rate must be positive";
          return result;
        }
        status_rate_ = rate;
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          std::bind(&MotorControllerNode::publish_motor_status, this));
        RCLCPP_INFO(this->get_logger(), "Status rate changed to %.1f Hz", status_rate_);
      }
    }

    return result;
  }

  void on_command(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_cmd_ = *msg;
    has_cmd_ = true;
  }

  void publish_motor_status()
  {
    msg_count_++;

    // Check for complete failure
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_diagnostics("MOTOR_FAILURE", "Motor failure (injected)");
      return;
    }

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"left_wheel", "right_wheel"};

    // Get latest command
    double linear_x = 0.0;
    double angular_z = 0.0;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (has_cmd_) {
        linear_x = latest_cmd_.linear.x;
        angular_z = latest_cmd_.angular.z;
      }
    }

    // Differential drive conversion
    double left_vel = linear_x - angular_z;
    double right_vel = linear_x + angular_z;

    msg.velocity = {left_vel, right_vel};

    // Effort with Gaussian noise
    double left_effort = left_vel + normal_dist_(rng_) * torque_noise_;
    double right_effort = right_vel + normal_dist_(rng_) * torque_noise_;
    msg.effort = {left_effort, right_effort};

    // Position placeholder (not tracked here - joint_driver accumulates)
    msg.position = {0.0, 0.0};

    motor_pub_->publish(msg);

    // Diagnostics
    if (!has_cmd_) {
      publish_diagnostics("NO_COMMAND", "No command received yet");
    } else if (torque_noise_ > 0.1) {
      publish_diagnostics("HIGH_NOISE", "High torque noise: " + std::to_string(torque_noise_));
    } else {
      publish_diagnostics("OK", "Operating normally");
    }
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "motor_controller";
    diag.hardware_id = "actuation_motors";

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

    kv.key = "torque_noise";
    kv.value = std::to_string(torque_noise_);
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Latest command (protected by mutex for thread safety)
  std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  bool has_cmd_{false};

  // Parameters
  double torque_noise_;
  double failure_probability_;
  double status_rate_;

  // Statistics
  uint64_t msg_count_{0};
};

}  // namespace actuation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<actuation::MotorControllerNode>());
  rclcpp::shutdown();
  return 0;
}
