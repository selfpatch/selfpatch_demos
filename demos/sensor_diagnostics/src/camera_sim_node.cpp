// Copyright 2026 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file camera_sim_node.cpp
/// @brief Simulated camera sensor with configurable fault injection
///
/// Publishes simulated Image and CameraInfo messages
/// and supports runtime fault injection via ROS 2 parameters.
/// Diagnostics are published to /diagnostics for the legacy fault reporting path
/// via ros2_medkit_diagnostic_bridge.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace sensor_diagnostics
{

class CameraSimNode : public rclcpp::Node
{
public:
  CameraSimNode()
  : Node("camera_sim"),
    rng_(std::random_device{}()),
    uniform_dist_(0.0, 1.0)
  {
    // Declare parameters
    this->declare_parameter("rate", 30.0);          // Hz
    this->declare_parameter("width", 640);          // pixels
    this->declare_parameter("height", 480);         // pixels
    this->declare_parameter("noise_level", 0.0);    // 0.0 - 1.0 (fraction of pixels with noise)
    this->declare_parameter("failure_probability", 0.0);
    this->declare_parameter("inject_black_frames", false);
    this->declare_parameter("brightness", 128);     // 0-255 base brightness

    load_parameters();

    // Create publishers
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    // Publish to absolute /diagnostics topic for legacy fault reporting via diagnostic_bridge
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Create timer (with rate validation)
    double rate = this->get_parameter("rate").as_double();
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'rate' must be > 0.0 Hz, but got %.3f. Falling back to 30.0 Hz.", rate);
      rate = 30.0;
      this->set_parameters({rclcpp::Parameter("rate", rate)});
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CameraSimNode::publish_image, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CameraSimNode::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "Camera simulator started at %.1f Hz (%dx%d)",
      rate, width_, height_);
  }

private:
  void load_parameters()
  {
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    noise_level_ = this->get_parameter("noise_level").as_double();
    failure_probability_ = this->get_parameter("failure_probability").as_double();
    inject_black_frames_ = this->get_parameter("inject_black_frames").as_bool();
    brightness_ = this->get_parameter("brightness").as_int();
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "noise_level") {
        noise_level_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Noise level changed to %.2f", noise_level_);
      } else if (param.get_name() == "failure_probability") {
        failure_probability_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Failure probability changed to %.2f",
          failure_probability_);
      } else if (param.get_name() == "inject_black_frames") {
        inject_black_frames_ = param.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Black frames %s",
          inject_black_frames_ ? "enabled" : "disabled");
      } else if (param.get_name() == "brightness") {
        brightness_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Brightness changed to %d", brightness_);
      }
    }

    return result;
  }

  void publish_image()
  {
    msg_count_++;

    // Check for complete failure
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_diagnostics("TIMEOUT", "Camera failure (injected)");
      return;
    }

    auto image = sensor_msgs::msg::Image();
    image.header.stamp = this->now();
    image.header.frame_id = "camera_link";

    image.width = width_;
    image.height = height_;
    image.encoding = "rgb8";
    image.is_bigendian = false;
    image.step = width_ * 3;  // 3 bytes per pixel (RGB)

    // Generate image data
    size_t data_size = static_cast<size_t>(width_ * height_ * 3);
    image.data.resize(data_size);

    bool is_black_frame = inject_black_frames_ && uniform_dist_(rng_) < 0.1;

    if (is_black_frame) {
      // All black frame
      std::fill(image.data.begin(), image.data.end(), 0);
      publish_diagnostics("BLACK_FRAME", "Black frame detected");
    } else {
      // Generate gradient pattern with noise
      for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
          size_t idx = static_cast<size_t>((y * width_ + x) * 3);

          // Base gradient pattern
          uint8_t r = static_cast<uint8_t>(std::clamp(brightness_ + x / 5, 0, 255));
          uint8_t g = static_cast<uint8_t>(std::clamp(brightness_ + y / 4, 0, 255));
          uint8_t b = static_cast<uint8_t>(std::clamp(brightness_, 0, 255));

          // Add noise
          if (uniform_dist_(rng_) < noise_level_) {
            r = static_cast<uint8_t>(uniform_dist_(rng_) * 255);
            g = static_cast<uint8_t>(uniform_dist_(rng_) * 255);
            b = static_cast<uint8_t>(uniform_dist_(rng_) * 255);
          }

          image.data[idx] = r;
          image.data[idx + 1] = g;
          image.data[idx + 2] = b;
        }
      }

      if (noise_level_ > 0.1) {
        publish_diagnostics("HIGH_NOISE", "High noise level: " + std::to_string(noise_level_));
      } else if (brightness_ < 30) {
        publish_diagnostics("LOW_BRIGHTNESS", "Image too dark");
      } else if (brightness_ > 225) {
        publish_diagnostics("OVEREXPOSED", "Image overexposed");
      } else {
        publish_diagnostics("OK", "Operating normally");
      }
    }

    image_pub_->publish(image);

    // Publish camera info
    auto info = sensor_msgs::msg::CameraInfo();
    info.header = image.header;
    info.width = width_;
    info.height = height_;

    // Simple pinhole camera model
    double fx = 500.0;  // focal length x
    double fy = 500.0;  // focal length y
    double cx = width_ / 2.0;   // principal point x
    double cy = height_ / 2.0;  // principal point y

    info.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    info.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
    info.distortion_model = "plumb_bob";
    info.d = {0, 0, 0, 0, 0};  // No distortion

    info_pub_->publish(info);
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "camera_sim";
    diag.hardware_id = "camera_sensor";

    if (status == "OK") {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (status == "HIGH_NOISE" || status == "LOW_BRIGHTNESS" || status == "OVEREXPOSED") {
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

    kv.key = "resolution";
    kv.value = std::to_string(width_) + "x" + std::to_string(height_);
    diag.values.push_back(kv);

    kv.key = "noise_level";
    kv.value = std::to_string(noise_level_);
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Parameters
  int width_;
  int height_;
  double noise_level_;
  double failure_probability_;
  bool inject_black_frames_;
  int brightness_;

  // Statistics
  uint64_t msg_count_{0};
};

}  // namespace sensor_diagnostics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_diagnostics::CameraSimNode>());
  rclcpp::shutdown();
  return 0;
}
