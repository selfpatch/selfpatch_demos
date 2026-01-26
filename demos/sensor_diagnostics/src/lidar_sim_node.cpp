// Copyright 2026 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file lidar_sim_node.cpp
/// @brief Simulated LiDAR sensor with configurable fault injection
///
/// This node publishes simulated LaserScan messages with realistic patterns
/// and supports runtime fault injection via ROS 2 parameters.
/// Diagnostics are published to /diagnostics for the legacy fault reporting path
/// via ros2_medkit_diagnostic_bridge.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace sensor_diagnostics
{

class LidarSimNode : public rclcpp::Node
{
public:
  LidarSimNode()
  : Node("lidar_sim"),
    rng_(std::random_device{}()),
    normal_dist_(0.0, 1.0),
    uniform_dist_(0.0, 1.0),
    start_time_(this->now())
  {
    // Declare parameters with defaults
    this->declare_parameter("scan_rate", 10.0);         // Hz
    this->declare_parameter("range_min", 0.12);         // meters
    this->declare_parameter("range_max", 3.5);          // meters
    this->declare_parameter("angle_min", -3.14159);     // radians
    this->declare_parameter("angle_max", 3.14159);      // radians
    this->declare_parameter("num_readings", 360);       // number of laser beams
    this->declare_parameter("noise_stddev", 0.01);      // meters
    this->declare_parameter("failure_probability", 0.0);  // 0.0 - 1.0
    this->declare_parameter("inject_nan", false);
    this->declare_parameter("drift_rate", 0.0);         // meters per second

    // Load parameters
    load_parameters();

    // Create publisher
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    // Publish to absolute /diagnostics topic for legacy fault reporting via diagnostic_bridge
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Calculate publish period from rate (with validation)
    double rate = this->get_parameter("scan_rate").as_double();
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid scan_rate parameter value (%f Hz). Using default 10.0 Hz instead.",
        rate);
      rate = 10.0;
      this->set_parameters({rclcpp::Parameter("scan_rate", rate)});
    }
    auto period = std::chrono::duration<double>(1.0 / rate);

    // Create timer
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LidarSimNode::publish_scan, this));

    // Register parameter callback for runtime reconfiguration
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&LidarSimNode::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "LiDAR simulator started at %.1f Hz", rate);
  }

private:
  void load_parameters()
  {
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    num_readings_ = this->get_parameter("num_readings").as_int();
    if (num_readings_ <= 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid num_readings parameter (%d). Using default 360.",
        num_readings_);
      num_readings_ = 360;
      this->set_parameters({rclcpp::Parameter("num_readings", num_readings_)});
    }
    noise_stddev_ = this->get_parameter("noise_stddev").as_double();
    failure_probability_ = this->get_parameter("failure_probability").as_double();
    inject_nan_ = this->get_parameter("inject_nan").as_bool();
    drift_rate_ = this->get_parameter("drift_rate").as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "noise_stddev") {
        noise_stddev_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Noise stddev changed to %.4f", noise_stddev_);
      } else if (param.get_name() == "failure_probability") {
        failure_probability_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Failure probability changed to %.2f",
          failure_probability_);
      } else if (param.get_name() == "inject_nan") {
        inject_nan_ = param.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Inject NaN %s",
          inject_nan_ ? "enabled" : "disabled");
      } else if (param.get_name() == "drift_rate") {
        drift_rate_ = param.as_double();
        // Reset start time for demo mode - drift accumulates from parameter change
        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Drift rate changed to %.4f (timer reset)", drift_rate_);
      } else if (param.get_name() == "scan_rate") {
        // Update timer with new rate (with validation)
        double rate = param.as_double();
        if (rate <= 0.0) {
          RCLCPP_WARN(
            this->get_logger(),
            "Invalid scan_rate parameter value (%f Hz). Rejecting change.",
            rate);
          result.successful = false;
          result.reason = "scan_rate must be positive";
          return result;
        }
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          std::bind(&LidarSimNode::publish_scan, this));
        RCLCPP_INFO(this->get_logger(), "Scan rate changed to %.1f Hz", rate);
      }
    }

    return result;
  }

  void publish_scan()
  {
    msg_count_++;

    // Check for complete failure
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_diagnostics("TIMEOUT", "Sensor failure (injected)");
      return;  // Don't publish - simulates timeout
    }

    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = this->now();
    msg.header.frame_id = "lidar_link";

    msg.angle_min = angle_min_;
    msg.angle_max = angle_max_;
    msg.angle_increment = (angle_max_ - angle_min_) / static_cast<double>(num_readings_);
    msg.time_increment = 0.0;
    msg.scan_time = 1.0 / this->get_parameter("scan_rate").as_double();
    msg.range_min = range_min_;
    msg.range_max = range_max_;

    // Calculate drift offset
    double elapsed = (this->now() - start_time_).seconds();
    double drift_offset = drift_rate_ * elapsed;

    // Generate simulated ranges with noise
    msg.ranges.resize(num_readings_);
    msg.intensities.resize(num_readings_);

    int nan_count = 0;
    for (int i = 0; i < num_readings_; i++) {
      // Base simulated pattern - circular room with some objects
      double angle = angle_min_ + i * msg.angle_increment;
      double base_range = generate_simulated_range(angle);

      // Apply drift
      base_range += drift_offset;

      // Add noise
      double noise = normal_dist_(rng_) * noise_stddev_;
      double range = base_range + noise;

      // Clamp to valid range
      range = std::clamp(range, range_min_, range_max_);

      // Inject NaN if configured (100% rate for clear fault demonstration)
      if (inject_nan_) {
        range = std::numeric_limits<double>::quiet_NaN();
        nan_count++;
      }

      msg.ranges[i] = static_cast<float>(range);
      msg.intensities[i] = static_cast<float>(100.0 * (1.0 - range / range_max_));
    }

    scan_pub_->publish(msg);

    // Publish diagnostics
    if (nan_count > 0) {
      publish_diagnostics("NAN_VALUES", "NaN values detected: " + std::to_string(nan_count));
    } else if (noise_stddev_ > 0.1) {
      publish_diagnostics("HIGH_NOISE", "Noise stddev: " + std::to_string(noise_stddev_));
    } else if (drift_offset > 0.1) {
      publish_diagnostics("DRIFTING", "Drift: " + std::to_string(drift_offset) + "m");
    } else {
      publish_diagnostics("OK", "Operating normally");
    }
  }

  double generate_simulated_range(double angle)
  {
    // Simulate a room with walls at ~2.5m and some objects
    double room_distance = 2.5;

    // Add some "objects" at fixed angles
    if (angle > 0.5 && angle < 0.8) {
      return 1.2;  // Object on the right
    }
    if (angle > -1.2 && angle < -0.9) {
      return 1.8;  // Object on the left
    }
    if (angle > 2.8 || angle < -2.8) {
      return 1.5;  // Object behind
    }

    // Default room walls
    return room_distance + normal_dist_(rng_) * 0.05;
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "lidar_sim";
    diag.hardware_id = "lidar_sensor";

    if (status == "OK") {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else {
      // All non-OK statuses are ERROR level for clear fault reporting
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    diag.message = message;

    // Add key-value pairs
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "status";
    kv.value = status;
    diag.values.push_back(kv);

    kv.key = "msg_count";
    kv.value = std::to_string(msg_count_);
    diag.values.push_back(kv);

    kv.key = "noise_stddev";
    kv.value = std::to_string(noise_stddev_);
    diag.values.push_back(kv);

    kv.key = "failure_probability";
    kv.value = std::to_string(failure_probability_);
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Parameters
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  int num_readings_;
  double noise_stddev_;
  double failure_probability_;
  bool inject_nan_;
  double drift_rate_;

  // Statistics
  rclcpp::Time start_time_;
  uint64_t msg_count_{0};
};

}  // namespace sensor_diagnostics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_diagnostics::LidarSimNode>());
  rclcpp::shutdown();
  return 0;
}
