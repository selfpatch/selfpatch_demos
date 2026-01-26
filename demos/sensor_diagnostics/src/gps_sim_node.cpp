// Copyright 2026 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file gps_sim_node.cpp
/// @brief Simulated GPS sensor with configurable fault injection
///
/// Publishes simulated NavSatFix messages with realistic GPS patterns
/// and supports runtime fault injection via ROS 2 parameters.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace sensor_diagnostics
{

class GpsSimNode : public rclcpp::Node
{
public:
  GpsSimNode()
  : Node("gps_sim"),
    rng_(std::random_device{}()),
    normal_dist_(0.0, 1.0),
    uniform_dist_(0.0, 1.0),
    start_time_(this->now())
  {
    // Declare parameters
    this->declare_parameter("rate", 1.0);                 // Hz (GPS typically 1-10 Hz)
    this->declare_parameter("base_latitude", 52.2297);    // Warsaw, Poland
    this->declare_parameter("base_longitude", 21.0122);
    this->declare_parameter("base_altitude", 100.0);      // meters
    this->declare_parameter("position_noise_stddev", 2.0);  // meters (CEP)
    this->declare_parameter("altitude_noise_stddev", 5.0);  // meters
    this->declare_parameter("failure_probability", 0.0);
    this->declare_parameter("inject_nan", false);
    this->declare_parameter("drift_rate", 0.0);           // meters/second

    load_parameters();

    // Create publishers
    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
    // Publish to absolute /diagnostics topic for legacy fault reporting via diagnostic_bridge
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Create timer (with rate validation)
    double rate = this->get_parameter("rate").as_double();
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid GPS publish rate (%f Hz) configured; using default of 1.0 Hz instead.",
        rate);
      rate = 1.0;
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GpsSimNode::publish_fix, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&GpsSimNode::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "GPS simulator started at %.1f Hz", rate);
  }

private:
  void load_parameters()
  {
    base_latitude_ = this->get_parameter("base_latitude").as_double();
    base_longitude_ = this->get_parameter("base_longitude").as_double();
    base_altitude_ = this->get_parameter("base_altitude").as_double();
    position_noise_stddev_ = this->get_parameter("position_noise_stddev").as_double();
    altitude_noise_stddev_ = this->get_parameter("altitude_noise_stddev").as_double();
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
      if (param.get_name() == "position_noise_stddev") {
        position_noise_stddev_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Position noise changed to %.2f m",
          position_noise_stddev_);
      } else if (param.get_name() == "altitude_noise_stddev") {
        altitude_noise_stddev_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Altitude noise changed to %.2f m",
          altitude_noise_stddev_);
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
      }
    }

    return result;
  }

  void publish_fix()
  {
    msg_count_++;

    // Check for complete failure (no satellite fix)
    if (uniform_dist_(rng_) < failure_probability_) {
      // Publish message with NO_FIX status
      auto msg = sensor_msgs::msg::NavSatFix();
      msg.header.stamp = this->now();
      msg.header.frame_id = "gps_link";
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      fix_pub_->publish(msg);
      publish_diagnostics("NO_FIX", "GPS signal lost (injected)");
      return;
    }

    auto msg = sensor_msgs::msg::NavSatFix();
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps_link";

    // GPS status - normal fix
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    // Calculate elapsed time for drift
    double elapsed = (this->now() - start_time_).seconds();
    double drift_offset = drift_rate_ * elapsed;

    // Convert drift from meters to degrees (approximately)
    // 1 degree latitude ≈ 111,000 meters
    // 1 degree longitude varies with latitude
    double lat_drift = drift_offset / 111000.0;
    double lon_drift = drift_offset / (111000.0 * std::cos(base_latitude_ * M_PI / 180.0));

    // Add noise to position (in meters, converted to degrees)
    double lat_noise = (normal_dist_(rng_) * position_noise_stddev_) / 111000.0;
    double lon_noise = (normal_dist_(rng_) * position_noise_stddev_) /
      (111000.0 * std::cos(base_latitude_ * M_PI / 180.0));
    double alt_noise = normal_dist_(rng_) * altitude_noise_stddev_;

    msg.latitude = base_latitude_ + lat_drift + lat_noise;
    msg.longitude = base_longitude_ + lon_drift + lon_noise;
    msg.altitude = base_altitude_ + alt_noise;

    // Covariance (diagonal, in m^2)
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    msg.position_covariance[0] = position_noise_stddev_ * position_noise_stddev_;
    msg.position_covariance[4] = position_noise_stddev_ * position_noise_stddev_;
    msg.position_covariance[8] = altitude_noise_stddev_ * altitude_noise_stddev_;

    // Inject NaN if configured (100% rate for clear fault demonstration)
    if (inject_nan_) {
      msg.latitude = std::numeric_limits<double>::quiet_NaN();
      publish_diagnostics("NAN_VALUES", "Invalid GPS coordinates");
    } else if (drift_offset > 5.0) {
      publish_diagnostics("DRIFTING", "Position drift: " + std::to_string(drift_offset) + "m");
    } else if (position_noise_stddev_ > 10.0) {
      publish_diagnostics("LOW_ACCURACY", "High position noise: " +
        std::to_string(position_noise_stddev_) + "m");
    } else {
      publish_diagnostics("OK", "GPS fix acquired");
    }

    fix_pub_->publish(msg);
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "gps_sim";
    diag.hardware_id = "gps_sensor";

    if (status == "OK") {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else {
      // All non-OK statuses are ERROR level for clear fault reporting
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

    kv.key = "position_noise";
    kv.value = std::to_string(position_noise_stddev_) + "m";
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Parameters
  double base_latitude_;
  double base_longitude_;
  double base_altitude_;
  double position_noise_stddev_;
  double altitude_noise_stddev_;
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
  rclcpp::spin(std::make_shared<sensor_diagnostics::GpsSimNode>());
  rclcpp::shutdown();
  return 0;
}
