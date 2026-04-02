// Copyright 2026 selfpatch
// Licensed under the Apache License, Version 2.0

/// @file behavior_planner.cpp
/// @brief Behavior planner node for the Planning ECU
///
/// Subscribes to a path (nav_msgs/msg/Path) and publishes velocity commands
/// (geometry_msgs/msg/Twist) using simple proportional control toward the next
/// waypoint. Supports direction inversion and failure probability for fault
/// injection.

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace multi_ecu_demo
{

class BehaviorPlanner : public rclcpp::Node
{
public:
  BehaviorPlanner()
  : Node("behavior_planner"),
    rng_(std::random_device{}()),
    uniform_dist_(0.0, 1.0)
  {
    // Declare parameters
    this->declare_parameter("inject_wrong_direction", false);
    this->declare_parameter("failure_probability", 0.0);
    this->declare_parameter("command_rate", 10.0);  // Hz

    load_parameters();

    // Create publishers
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("commands", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Create subscription to path
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "path", 10,
      std::bind(&BehaviorPlanner::on_path, this, std::placeholders::_1));

    // Create timer (with rate validation)
    double rate = command_rate_;
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'command_rate' must be > 0.0 Hz, but got %.3f. Falling back to 10.0 Hz.",
        rate);
      rate = 10.0;
      this->set_parameters({rclcpp::Parameter("command_rate", rate)});
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BehaviorPlanner::compute_command, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&BehaviorPlanner::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Behavior planner started at %.1f Hz", rate);
  }

private:
  void load_parameters()
  {
    inject_wrong_direction_ = this->get_parameter("inject_wrong_direction").as_bool();
    failure_probability_ = this->get_parameter("failure_probability").as_double();
    command_rate_ = this->get_parameter("command_rate").as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "inject_wrong_direction") {
        inject_wrong_direction_ = param.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Wrong direction %s",
          inject_wrong_direction_ ? "enabled" : "disabled");
      } else if (param.get_name() == "failure_probability") {
        failure_probability_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Failure probability changed to %.2f", failure_probability_);
      } else if (param.get_name() == "command_rate") {
        double rate = param.as_double();
        if (rate <= 0.0) {
          result.successful = false;
          result.reason = "command_rate must be positive";
          return result;
        }
        command_rate_ = rate;
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          std::bind(&BehaviorPlanner::compute_command, this));
        RCLCPP_INFO(this->get_logger(), "Command rate changed to %.1f Hz", command_rate_);
      }
    }

    return result;
  }

  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    current_path_ = *msg;
    // Reset waypoint index when a new path arrives
    waypoint_index_ = 0;
  }

  void compute_command()
  {
    cmd_count_++;

    // Check for failure injection
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_diagnostics("COMMAND_FAILURE", "Command generation failed (injected)");
      return;
    }

    // No path received yet
    if (current_path_.poses.empty()) {
      publish_diagnostics("NO_PATH", "Waiting for path input");
      return;
    }

    // Get current target waypoint
    const auto & target = current_path_.poses[waypoint_index_].pose;

    // Simple proportional control toward the waypoint
    // Assume current position is origin (0,0) for simplicity in this simulation
    double dx = target.position.x;
    double dy = target.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double heading = std::atan2(dy, dx);

    auto cmd = geometry_msgs::msg::Twist();

    // Proportional gains
    constexpr double kp_linear = 0.5;
    constexpr double kp_angular = 1.0;

    // Linear velocity toward waypoint
    cmd.linear.x = kp_linear * distance;

    // Angular velocity for heading correction
    cmd.angular.z = kp_angular * heading;

    // Inject wrong direction: negate linear.x
    if (inject_wrong_direction_) {
      cmd.linear.x = -cmd.linear.x;
    }

    cmd_pub_->publish(cmd);

    // Advance to next waypoint (loop path)
    waypoint_index_ = (waypoint_index_ + 1) % static_cast<int>(current_path_.poses.size());

    // Diagnostics
    if (inject_wrong_direction_) {
      publish_diagnostics(
        "WRONG_DIRECTION", "Driving in reverse direction (injected)");
    } else if (distance > 10.0) {
      publish_diagnostics(
        "FAR_FROM_WAYPOINT",
        "Distance to waypoint: " + std::to_string(distance) + " m");
    } else {
      publish_diagnostics("OK", "Operating normally");
    }
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "behavior_planner";
    diag.hardware_id = "planning_ecu";

    if (status == "OK") {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    diag.message = message;

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "status";
    kv.value = status;
    diag.values.push_back(kv);

    kv.key = "cmd_count";
    kv.value = std::to_string(cmd_count_);
    diag.values.push_back(kv);

    kv.key = "waypoint_index";
    kv.value = std::to_string(waypoint_index_);
    diag.values.push_back(kv);

    kv.key = "path_length";
    kv.value = std::to_string(current_path_.poses.size());
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Subscription
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Parameters
  bool inject_wrong_direction_;
  double failure_probability_;
  double command_rate_;

  // State
  nav_msgs::msg::Path current_path_;
  int waypoint_index_{0};
  uint64_t cmd_count_{0};
};

}  // namespace multi_ecu_demo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_ecu_demo::BehaviorPlanner>());
  rclcpp::shutdown();
  return 0;
}
