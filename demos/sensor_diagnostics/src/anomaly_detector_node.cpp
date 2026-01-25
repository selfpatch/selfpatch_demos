// Copyright 2025 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file anomaly_detector_node.cpp
/// @brief Monitors sensor streams and reports detected anomalies as faults
///
/// Subscribes to sensor topics and diagnostics, detects anomalies,
/// and publishes fault events.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace sensor_diagnostics
{

class AnomalyDetectorNode : public rclcpp::Node
{
public:
  AnomalyDetectorNode()
  : Node("anomaly_detector")
  {
    // Declare parameters for thresholds
    this->declare_parameter("rate_timeout_sec", 5.0);
    this->declare_parameter("max_nan_ratio", 0.1);
    this->declare_parameter("lidar_rate_min", 5.0);
    this->declare_parameter("imu_rate_min", 50.0);
    this->declare_parameter("gps_rate_min", 0.5);

    rate_timeout_sec_ = this->get_parameter("rate_timeout_sec").as_double();
    max_nan_ratio_ = this->get_parameter("max_nan_ratio").as_double();

    // Create subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sensors/lidar_sim/scan", 10,
      std::bind(&AnomalyDetectorNode::lidar_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/sensors/imu_sim/imu", 10,
      std::bind(&AnomalyDetectorNode::imu_callback, this, std::placeholders::_1));

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/sensors/gps_sim/fix", 10,
      std::bind(&AnomalyDetectorNode::gps_callback, this, std::placeholders::_1));

    // Create publisher for detected faults
    fault_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "detected_faults", 10);

    // Timer for periodic health check
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AnomalyDetectorNode::check_sensor_health, this));

    RCLCPP_INFO(this->get_logger(), "Anomaly detector started");
  }

private:
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_timestamps_["lidar"] = this->now();
    lidar_msg_count_++;

    // Check for NaN values
    int nan_count = 0;
    for (const auto & range : msg->ranges) {
      if (std::isnan(range)) {
        nan_count++;
      }
    }

    double nan_ratio = static_cast<double>(nan_count) / static_cast<double>(msg->ranges.size());
    if (nan_ratio > max_nan_ratio_) {
      report_fault("lidar_sim", "SENSOR_NAN",
        "LiDAR has " + std::to_string(static_cast<int>(nan_ratio * 100)) + "% NaN values");
    }

    // Check for all-zero ranges (sensor malfunction)
    bool all_min = true;
    for (const auto & range : msg->ranges) {
      if (!std::isnan(range) && range > msg->range_min + 0.01) {
        all_min = false;
        break;
      }
    }
    if (all_min && !msg->ranges.empty()) {
      report_fault("lidar_sim", "SENSOR_MALFUNCTION", "LiDAR returns all minimum range values");
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_timestamps_["imu"] = this->now();
    imu_msg_count_++;

    // Check for NaN values
    if (std::isnan(msg->linear_acceleration.x) ||
      std::isnan(msg->linear_acceleration.y) ||
      std::isnan(msg->linear_acceleration.z))
    {
      report_fault("imu_sim", "SENSOR_NAN", "IMU acceleration contains NaN values");
    }

    if (std::isnan(msg->angular_velocity.x) ||
      std::isnan(msg->angular_velocity.y) ||
      std::isnan(msg->angular_velocity.z))
    {
      report_fault("imu_sim", "SENSOR_NAN", "IMU angular velocity contains NaN values");
    }

    // Check for unrealistic acceleration (should be ~9.81 on z when stationary)
    double accel_magnitude = std::sqrt(
      msg->linear_acceleration.x * msg->linear_acceleration.x +
      msg->linear_acceleration.y * msg->linear_acceleration.y +
      msg->linear_acceleration.z * msg->linear_acceleration.z);

    if (accel_magnitude < 5.0 || accel_magnitude > 15.0) {
      report_fault("imu_sim", "SENSOR_OUT_OF_RANGE",
        "IMU acceleration magnitude: " + std::to_string(accel_magnitude) + " m/s^2");
    }
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    sensor_timestamps_["gps"] = this->now();
    gps_msg_count_++;

    // Check for no fix
    if (msg->status.status < 0) {
      report_fault("gps_sim", "NO_FIX", "GPS has no satellite fix");
      return;
    }

    // Check for NaN coordinates
    if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) {
      report_fault("gps_sim", "SENSOR_NAN", "GPS coordinates contain NaN values");
    }

    // Check for unrealistic coordinates
    if (msg->latitude < -90 || msg->latitude > 90 ||
      msg->longitude < -180 || msg->longitude > 180)
    {
      report_fault("gps_sim", "SENSOR_OUT_OF_RANGE", "GPS coordinates out of valid range");
    }
  }

  void check_sensor_health()
  {
    auto now = this->now();

    // Check for sensor timeouts
    check_timeout("lidar", now);
    check_timeout("imu", now);
    check_timeout("gps", now);

    // Calculate and log rates
    double elapsed = 1.0;  // 1 second timer
    double lidar_rate = (lidar_msg_count_ - last_lidar_count_) / elapsed;
    double imu_rate = (imu_msg_count_ - last_imu_count_) / elapsed;
    double gps_rate = (gps_msg_count_ - last_gps_count_) / elapsed;

    last_lidar_count_ = lidar_msg_count_;
    last_imu_count_ = imu_msg_count_;
    last_gps_count_ = gps_msg_count_;

    // Check for degraded rates
    double lidar_rate_min = this->get_parameter("lidar_rate_min").as_double();
    double imu_rate_min = this->get_parameter("imu_rate_min").as_double();
    double gps_rate_min = this->get_parameter("gps_rate_min").as_double();

    if (lidar_rate > 0 && lidar_rate < lidar_rate_min) {
      report_fault("lidar_sim", "RATE_DEGRADED",
        "LiDAR rate: " + std::to_string(lidar_rate) + " Hz (min: " +
        std::to_string(lidar_rate_min) + ")");
    }

    if (imu_rate > 0 && imu_rate < imu_rate_min) {
      report_fault("imu_sim", "RATE_DEGRADED",
        "IMU rate: " + std::to_string(imu_rate) + " Hz (min: " +
        std::to_string(imu_rate_min) + ")");
    }

    if (gps_rate > 0 && gps_rate < gps_rate_min) {
      report_fault("gps_sim", "RATE_DEGRADED",
        "GPS rate: " + std::to_string(gps_rate) + " Hz (min: " +
        std::to_string(gps_rate_min) + ")");
    }
  }

  void check_timeout(const std::string & sensor, const rclcpp::Time & now)
  {
    auto it = sensor_timestamps_.find(sensor);
    if (it != sensor_timestamps_.end()) {
      double elapsed = (now - it->second).seconds();
      if (elapsed > rate_timeout_sec_) {
        report_fault(sensor + "_sim", "SENSOR_TIMEOUT",
          sensor + " no messages for " + std::to_string(elapsed) + "s");
      }
    }
  }

  void report_fault(
    const std::string & source, const std::string & code,
    const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto status = diagnostic_msgs::msg::DiagnosticStatus();
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.name = source;
    status.message = message;
    status.hardware_id = source;

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "fault_code";
    kv.value = code;
    status.values.push_back(kv);

    kv.key = "timestamp";
    kv.value = std::to_string(this->now().nanoseconds());
    status.values.push_back(kv);

    diag_array.status.push_back(status);
    fault_pub_->publish(diag_array);

    RCLCPP_WARN(this->get_logger(), "[%s] %s: %s", source.c_str(), code.c_str(), message.c_str());
  }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  // Publisher
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr fault_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double rate_timeout_sec_;
  double max_nan_ratio_;

  // State tracking
  std::map<std::string, rclcpp::Time> sensor_timestamps_;
  uint64_t lidar_msg_count_{0};
  uint64_t imu_msg_count_{0};
  uint64_t gps_msg_count_{0};
  uint64_t last_lidar_count_{0};
  uint64_t last_imu_count_{0};
  uint64_t last_gps_count_{0};
};

}  // namespace sensor_diagnostics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_diagnostics::AnomalyDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
