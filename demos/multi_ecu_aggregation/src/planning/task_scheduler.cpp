// Copyright 2026 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file task_scheduler.cpp
/// @brief Task scheduler node for the Planning ECU
///
/// Standalone timer node that cycles through task states
/// ("idle" -> "navigating" -> "executing" -> "returning" -> "idle").
/// Publishes the current state as a String message. Supports stuck-state
/// injection and failure probability for fault injection.

#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/string.hpp"

namespace multi_ecu_demo
{

class TaskScheduler : public rclcpp::Node
{
public:
  TaskScheduler()
  : Node("task_scheduler"),
    rng_(std::random_device{}()),
    uniform_dist_(0.0, 1.0)
  {
    // Declare parameters
    this->declare_parameter("inject_stuck", false);
    this->declare_parameter("failure_probability", 0.0);
    this->declare_parameter("schedule_rate", 1.0);  // Hz

    load_parameters();

    // Create publishers
    status_pub_ = this->create_publisher<std_msgs::msg::String>("task_status", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Create timer (with rate validation)
    double rate = schedule_rate_;
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'schedule_rate' must be > 0.0 Hz, but got %.3f. Falling back to 1.0 Hz.",
        rate);
      rate = 1.0;
      this->set_parameters({rclcpp::Parameter("schedule_rate", rate)});
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TaskScheduler::publish_task_status, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TaskScheduler::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Task scheduler started at %.1f Hz", rate);
  }

private:
  void load_parameters()
  {
    inject_stuck_ = this->get_parameter("inject_stuck").as_bool();
    failure_probability_ = this->get_parameter("failure_probability").as_double();
    schedule_rate_ = this->get_parameter("schedule_rate").as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "inject_stuck") {
        inject_stuck_ = param.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Stuck injection %s",
          inject_stuck_ ? "enabled" : "disabled");
      } else if (param.get_name() == "failure_probability") {
        failure_probability_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Failure probability changed to %.2f", failure_probability_);
      } else if (param.get_name() == "schedule_rate") {
        double rate = param.as_double();
        if (rate <= 0.0) {
          result.successful = false;
          result.reason = "schedule_rate must be positive";
          return result;
        }
        schedule_rate_ = rate;
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          std::bind(&TaskScheduler::publish_task_status, this));
        RCLCPP_INFO(this->get_logger(), "Schedule rate changed to %.1f Hz", schedule_rate_);
      }
    }

    return result;
  }

  void publish_task_status()
  {
    tick_count_++;

    // Check for failure injection
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_diagnostics("SCHEDULER_FAILURE", "Task scheduling failed (injected)");
      return;
    }

    // Get current state
    const std::string & current_state = task_states_[state_index_];

    // Publish current state
    auto msg = std_msgs::msg::String();
    msg.data = current_state;
    status_pub_->publish(msg);

    // Advance to next state (unless stuck)
    if (!inject_stuck_) {
      state_index_ = (state_index_ + 1) % static_cast<int>(task_states_.size());
    }

    // Diagnostics
    if (inject_stuck_) {
      publish_diagnostics(
        "STUCK", "Task stuck in state: " + current_state);
    } else {
      publish_diagnostics("OK", "Current state: " + current_state);
    }
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "task_scheduler";
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

    kv.key = "tick_count";
    kv.value = std::to_string(tick_count_);
    diag.values.push_back(kv);

    kv.key = "current_state";
    kv.value = task_states_[state_index_];
    diag.values.push_back(kv);

    kv.key = "state_index";
    kv.value = std::to_string(state_index_);
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Parameters
  bool inject_stuck_;
  double failure_probability_;
  double schedule_rate_;

  // State machine
  const std::vector<std::string> task_states_ = {
    "idle", "navigating", "executing", "returning"
  };
  int state_index_{0};
  uint64_t tick_count_{0};
};

}  // namespace multi_ecu_demo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_ecu_demo::TaskScheduler>());
  rclcpp::shutdown();
  return 0;
}
