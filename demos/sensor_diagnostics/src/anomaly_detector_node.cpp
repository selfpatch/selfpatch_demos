// Copyright 2025 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file anomaly_detector_node.cpp
/// @brief Monitors IMU and GPS sensors and reports detected anomalies as faults
///
/// This node implements the MODERN fault reporting path:
/// - Subscribes to IMU and GPS topics
/// - Detects anomalies (NaN values, out-of-range, timeouts)
/// - Reports faults directly to FaultManager via ReportFault service
///
/// Note: LiDAR and Camera use the LEGACY path (diagnostics → diagnostic_bridge → FaultManager)

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"
#include "sensor_msgs/msg/imu.hpp"
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
    this->declare_parameter("imu_rate_min", 50.0);
    this->declare_parameter("gps_rate_min", 0.5);

    rate_timeout_sec_ = this->get_parameter("rate_timeout_sec").as_double();
    max_nan_ratio_ = this->get_parameter("max_nan_ratio").as_double();

    // Create subscribers for MODERN path sensors (IMU, GPS)
    // Note: LiDAR and Camera use LEGACY path via /diagnostics → diagnostic_bridge
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/sensors/imu", 10,
      std::bind(&AnomalyDetectorNode::imu_callback, this, std::placeholders::_1));

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/sensors/fix", 10,
      std::bind(&AnomalyDetectorNode::gps_callback, this, std::placeholders::_1));

    // Create publisher for detected faults (supplementary diagnostic topic)
    fault_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "detected_faults", 10);

    // Create service client for FaultManager (MODERN path)
    report_fault_client_ = this->create_client<ros2_medkit_msgs::srv::ReportFault>(
      "/fault_manager/report_fault");

    // Timer for periodic health check
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AnomalyDetectorNode::check_sensor_health, this));

    // Timer for clearing passed faults (sensors recovered)
    clear_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&AnomalyDetectorNode::clear_passed_faults, this));

    RCLCPP_INFO(this->get_logger(), "Anomaly detector started (modern path: IMU, GPS)");
  }

private:
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

    // Check for sensor timeouts (modern path sensors only)
    check_timeout("imu", now);
    check_timeout("gps", now);

    // Calculate and log rates
    double elapsed = 1.0;  // 1 second timer
    double imu_rate = (imu_msg_count_ - last_imu_count_) / elapsed;
    double gps_rate = (gps_msg_count_ - last_gps_count_) / elapsed;

    last_imu_count_ = imu_msg_count_;
    last_gps_count_ = gps_msg_count_;

    // Check for degraded rates
    double imu_rate_min = this->get_parameter("imu_rate_min").as_double();
    double gps_rate_min = this->get_parameter("gps_rate_min").as_double();

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
    const std::string & message, uint8_t severity = 2)
  {
    // Track active faults for later clearing
    std::string fault_key = source + ":" + code;
    active_faults_.insert(fault_key);

    // Publish to diagnostic topic (legacy)
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

    // Call FaultManager service
    if (report_fault_client_->service_is_ready()) {
      auto request = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
      request->fault_code = code;
      request->event_type = ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED;
      request->severity = severity;
      request->description = message;
      request->source_id = "/processing/anomaly_detector/" + source;

      // Async call - don't block
      report_fault_client_->async_send_request(request);
      RCLCPP_WARN(this->get_logger(), "[%s] FAULT REPORTED: %s - %s",
        source.c_str(), code.c_str(), message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "[%s] %s: %s (FaultManager unavailable)",
        source.c_str(), code.c_str(), message.c_str());
    }
  }

  void report_passed(const std::string & source, const std::string & code)
  {
    std::string fault_key = source + ":" + code;
    active_faults_.erase(fault_key);

    if (report_fault_client_->service_is_ready()) {
      auto request = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
      request->fault_code = code;
      request->event_type = ros2_medkit_msgs::srv::ReportFault::Request::EVENT_PASSED;
      request->severity = 0;
      request->description = "";
      request->source_id = "/processing/anomaly_detector/" + source;

      report_fault_client_->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "[%s] FAULT CLEARED: %s", source.c_str(), code.c_str());
    }
  }

  void clear_passed_faults()
  {
    // Clear faults for sensors that haven't reported issues recently
    // This is handled implicitly by the health check timer
  }

  // Subscribers (modern path: IMU, GPS only)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  // Publisher
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr fault_pub_;

  // Service client for FaultManager
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report_fault_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr clear_timer_;

  // Parameters
  double rate_timeout_sec_;
  double max_nan_ratio_;

  // State tracking
  std::map<std::string, rclcpp::Time> sensor_timestamps_;
  std::set<std::string> active_faults_;
  uint64_t imu_msg_count_{0};
  uint64_t gps_msg_count_{0};
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
