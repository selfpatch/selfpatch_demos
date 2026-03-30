// Copyright 2026 selfpatch
// Licensed under the Apache License, Version 2.0

/// @file path_planner.cpp
/// @brief Path planner node for the Planning ECU
///
/// Subscribes to /perception/detections (Detection2DArray) and publishes a
/// 10-waypoint path in the "map" frame. Offsets the path when detections are
/// present to simulate obstacle avoidance. Supports artificial planning delay
/// and failure probability for fault injection.

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace multi_ecu_demo
{

class PathPlanner : public rclcpp::Node
{
public:
  PathPlanner()
  : Node("path_planner"),
    rng_(std::random_device{}()),
    uniform_dist_(0.0, 1.0)
  {
    // Declare parameters
    this->declare_parameter("planning_delay_ms", 0);
    this->declare_parameter("failure_probability", 0.0);
    this->declare_parameter("planning_rate", 5.0);  // Hz

    load_parameters();

    // Create publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Create subscription to perception detections (absolute topic)
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/perception/detections", 10,
      std::bind(&PathPlanner::on_detections, this, std::placeholders::_1));

    // Create timer (with rate validation)
    double rate = planning_rate_;
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'planning_rate' must be > 0.0 Hz, but got %.3f. Falling back to 5.0 Hz.",
        rate);
      rate = 5.0;
      this->set_parameters({rclcpp::Parameter("planning_rate", rate)});
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PathPlanner::plan_path, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PathPlanner::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Path planner started at %.1f Hz", rate);
  }

private:
  void load_parameters()
  {
    planning_delay_ms_ = this->get_parameter("planning_delay_ms").as_int();
    failure_probability_ = this->get_parameter("failure_probability").as_double();
    planning_rate_ = this->get_parameter("planning_rate").as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "planning_delay_ms") {
        planning_delay_ms_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Planning delay changed to %ld ms", planning_delay_ms_);
      } else if (param.get_name() == "failure_probability") {
        failure_probability_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Failure probability changed to %.2f", failure_probability_);
      } else if (param.get_name() == "planning_rate") {
        planning_rate_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Planning rate changed to %.1f Hz", planning_rate_);
      }
    }

    return result;
  }

  void on_detections(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    last_detection_count_ = static_cast<int>(msg->detections.size());
  }

  void plan_path()
  {
    plan_count_++;

    // Artificial planning delay
    if (planning_delay_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(planning_delay_ms_));
    }

    // Check for failure injection
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_diagnostics("PLANNING_FAILURE", "Path planning failed (injected)");
      return;
    }

    auto path = nav_msgs::msg::Path();
    path.header.stamp = this->now();
    path.header.frame_id = "map";

    // Generate 10-waypoint path
    constexpr int num_waypoints = 10;
    double lateral_offset = 0.0;

    // Offset path if detections exist (obstacle avoidance simulation)
    if (last_detection_count_ > 0) {
      lateral_offset = 1.0 * last_detection_count_;
    }

    for (int i = 0; i < num_waypoints; i++) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;

      // Straight-line path along x-axis with optional lateral offset on y-axis
      pose.pose.position.x = static_cast<double>(i) * 2.0;
      pose.pose.position.y = lateral_offset;
      pose.pose.position.z = 0.0;

      // Orientation: facing forward (identity quaternion)
      pose.pose.orientation.w = 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;

      path.poses.push_back(pose);
    }

    path_pub_->publish(path);

    // Diagnostics
    if (planning_delay_ms_ > 100) {
      publish_diagnostics(
        "SLOW_PLANNING",
        "Planning delay: " + std::to_string(planning_delay_ms_) + " ms");
    } else if (last_detection_count_ > 5) {
      publish_diagnostics(
        "HIGH_OBSTACLE_COUNT",
        "Avoiding " + std::to_string(last_detection_count_) + " detections");
    } else {
      publish_diagnostics("OK", "Operating normally");
    }
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "path_planner";
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

    kv.key = "plan_count";
    kv.value = std::to_string(plan_count_);
    diag.values.push_back(kv);

    kv.key = "detection_count";
    kv.value = std::to_string(last_detection_count_);
    diag.values.push_back(kv);

    kv.key = "planning_delay_ms";
    kv.value = std::to_string(planning_delay_ms_);
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Subscription
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Parameters
  int64_t planning_delay_ms_;
  double failure_probability_;
  double planning_rate_;

  // State
  int last_detection_count_{0};
  uint64_t plan_count_{0};
};

}  // namespace multi_ecu_demo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_ecu_demo::PathPlanner>());
  rclcpp::shutdown();
  return 0;
}
