// Copyright 2026 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file imu_sim_node.cpp
/// @brief Simulated IMU sensor with configurable fault injection
///
/// Publishes simulated Imu messages with realistic patterns
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
#include "sensor_msgs/msg/imu.hpp"

namespace sensor_diagnostics
{

class ImuSimNode : public rclcpp::Node
{
public:
  ImuSimNode()
  : Node("imu_sim"),
    rng_(std::random_device{}()),
    normal_dist_(0.0, 1.0),
    uniform_dist_(0.0, 1.0),
    start_time_(this->now())
  {
    // Declare parameters
    this->declare_parameter("rate", 100.0);               // Hz
    this->declare_parameter("accel_noise_stddev", 0.01);  // m/s^2
    this->declare_parameter("gyro_noise_stddev", 0.001);  // rad/s
    this->declare_parameter("failure_probability", 0.0);
    this->declare_parameter("inject_nan", false);
    this->declare_parameter("drift_rate", 0.0);           // rad/s drift

    load_parameters();

    // Create publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    // Publish to absolute /diagnostics topic for legacy fault reporting via diagnostic_bridge
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Create timer (with rate validation)
    double rate = this->get_parameter("rate").as_double();
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'rate' must be positive; using default 100.0 Hz instead of %.3f",
        rate);
      rate = 100.0;
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ImuSimNode::publish_imu, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ImuSimNode::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "IMU simulator started at %.1f Hz", rate);
  }

private:
  void load_parameters()
  {
    accel_noise_stddev_ = this->get_parameter("accel_noise_stddev").as_double();
    gyro_noise_stddev_ = this->get_parameter("gyro_noise_stddev").as_double();
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
      if (param.get_name() == "accel_noise_stddev") {
        accel_noise_stddev_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Accel noise changed to %.4f", accel_noise_stddev_);
      } else if (param.get_name() == "gyro_noise_stddev") {
        gyro_noise_stddev_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Gyro noise changed to %.4f", gyro_noise_stddev_);
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
        RCLCPP_INFO(this->get_logger(), "Drift rate changed to %.4f", drift_rate_);
      }
    }

    return result;
  }

  void publish_imu()
  {
    msg_count_++;

    // Check for complete failure
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_diagnostics("TIMEOUT", "Sensor failure (injected)");
      return;
    }

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    // Calculate elapsed time for drift
    double elapsed = (this->now() - start_time_).seconds();
    double drift_offset = drift_rate_ * elapsed;

    // Simulated stationary IMU with noise
    // Orientation (quaternion) - identity with small drift
    double yaw_drift = drift_offset;
    msg.orientation.w = std::cos(yaw_drift / 2.0);
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = std::sin(yaw_drift / 2.0);

    // Angular velocity - should be ~0 when stationary
    msg.angular_velocity.x = normal_dist_(rng_) * gyro_noise_stddev_;
    msg.angular_velocity.y = normal_dist_(rng_) * gyro_noise_stddev_;
    msg.angular_velocity.z = normal_dist_(rng_) * gyro_noise_stddev_ + drift_rate_;

    // Linear acceleration - gravity on z-axis when level
    msg.linear_acceleration.x = normal_dist_(rng_) * accel_noise_stddev_;
    msg.linear_acceleration.y = normal_dist_(rng_) * accel_noise_stddev_;
    msg.linear_acceleration.z = 9.81 + normal_dist_(rng_) * accel_noise_stddev_;

    // Covariance (diagonal)
    for (int i = 0; i < 9; i++) {
      msg.orientation_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
      msg.angular_velocity_covariance[i] =
        (i % 4 == 0) ? gyro_noise_stddev_ * gyro_noise_stddev_ : 0.0;
      msg.linear_acceleration_covariance[i] =
        (i % 4 == 0) ? accel_noise_stddev_ * accel_noise_stddev_ : 0.0;
    }

    // Inject NaN if configured
    if (inject_nan_ && uniform_dist_(rng_) < 0.05) {
      msg.linear_acceleration.x = std::numeric_limits<double>::quiet_NaN();
      publish_diagnostics("NAN_VALUES", "NaN values detected in acceleration");
    } else if (drift_offset > 0.1) {
      publish_diagnostics("DRIFTING", "Drift: " + std::to_string(drift_offset) + " rad");
    } else if (accel_noise_stddev_ > 0.1 || gyro_noise_stddev_ > 0.01) {
      publish_diagnostics("HIGH_NOISE", "High noise levels detected");
    } else {
      publish_diagnostics("OK", "Operating normally");
    }

    imu_pub_->publish(msg);
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "imu_sim";
    diag.hardware_id = "imu_sensor";

    if (status == "OK") {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (status == "HIGH_NOISE" || status == "DRIFTING") {
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

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
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
  double accel_noise_stddev_;
  double gyro_noise_stddev_;
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
  rclcpp::spin(std::make_shared<sensor_diagnostics::ImuSimNode>());
  rclcpp::shutdown();
  return 0;
}
