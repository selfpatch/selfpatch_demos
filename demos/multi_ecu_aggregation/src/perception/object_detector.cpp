// Copyright 2026 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file object_detector.cpp
/// @brief Timer-based object detector that subscribes to filtered point cloud
///
/// Generates fake 2D detections at a configurable rate. Subscribes to
/// filtered_points (PointCloud2) to track input availability. Supports
/// false positive injection, detection miss rate, and complete failure.
/// Diagnostics are published to /diagnostics.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace multi_ecu_demo
{

class ObjectDetector : public rclcpp::Node
{
public:
  ObjectDetector()
  : Node("object_detector"),
    rng_(std::random_device{}()),
    uniform_dist_(0.0, 1.0)
  {
    // Declare parameters with defaults
    this->declare_parameter("false_positive_rate", 0.0);  // 0.0 - 1.0
    this->declare_parameter("miss_rate", 0.0);            // 0.0 - 1.0
    this->declare_parameter("failure_probability", 0.0);  // 0.0 - 1.0
    this->declare_parameter("detection_rate", 5.0);       // Hz

    load_parameters();

    // Subscribe to filtered point cloud (tracks input availability)
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "filtered_points", 10,
      std::bind(&ObjectDetector::cloud_callback, this, std::placeholders::_1));

    // Create publishers
    detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
      "detections", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Create timer for detection rate (with validation)
    double rate = detection_rate_;
    if (rate <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid detection_rate parameter value (%f Hz). Using default 5.0 Hz.",
        rate);
      rate = 5.0;
      detection_rate_ = rate;
      this->set_parameters({rclcpp::Parameter("detection_rate", rate)});
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ObjectDetector::detect, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ObjectDetector::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Object detector started at %.1f Hz", rate);
  }

private:
  void load_parameters()
  {
    false_positive_rate_ = this->get_parameter("false_positive_rate").as_double();
    miss_rate_ = this->get_parameter("miss_rate").as_double();
    failure_probability_ = this->get_parameter("failure_probability").as_double();
    detection_rate_ = this->get_parameter("detection_rate").as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "false_positive_rate") {
        false_positive_rate_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "False positive rate changed to %.2f",
          false_positive_rate_);
      } else if (param.get_name() == "miss_rate") {
        miss_rate_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Miss rate changed to %.2f", miss_rate_);
      } else if (param.get_name() == "failure_probability") {
        failure_probability_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Failure probability changed to %.2f",
          failure_probability_);
      } else if (param.get_name() == "detection_rate") {
        double rate = param.as_double();
        if (rate <= 0.0) {
          RCLCPP_WARN(
            this->get_logger(),
            "Invalid detection_rate parameter value (%f Hz). Rejecting change.",
            rate);
          result.successful = false;
          result.reason = "detection_rate must be positive";
          return result;
        }
        detection_rate_ = rate;
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          std::bind(&ObjectDetector::detect, this));
        RCLCPP_INFO(this->get_logger(), "Detection rate changed to %.1f Hz", rate);
      }
    }

    return result;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_cloud_time_ = this->now();
    last_cloud_points_ = msg->width * msg->height;
  }

  void detect()
  {
    msg_count_++;

    // Check for complete failure
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_diagnostics("FAILURE", "Detector failure (injected)");
      return;
    }

    // Check if we have recent point cloud input
    bool has_input = false;
    if (last_cloud_time_.nanoseconds() > 0) {
      double elapsed = (this->now() - last_cloud_time_).seconds();
      has_input = elapsed < 2.0;  // Consider stale after 2 seconds
    }

    // Check for miss rate - suppress all detections
    if (uniform_dist_(rng_) < miss_rate_) {
      auto det_array = vision_msgs::msg::Detection2DArray();
      det_array.header.stamp = this->now();
      det_array.header.frame_id = "camera_link";
      detection_pub_->publish(det_array);
      publish_diagnostics("MISSED", "All detections suppressed (miss rate)");
      return;
    }

    auto det_array = vision_msgs::msg::Detection2DArray();
    det_array.header.stamp = this->now();
    det_array.header.frame_id = "camera_link";

    // Generate 1-3 fake detections
    std::uniform_int_distribution<int> count_dist(1, 3);
    int num_detections = count_dist(rng_);

    // Object class labels for fake detections
    static const std::vector<std::string> class_labels = {
      "person", "car", "bicycle", "obstacle", "cone"
    };
    std::uniform_int_distribution<size_t> class_dist(0, class_labels.size() - 1);

    for (int i = 0; i < num_detections; i++) {
      auto det = vision_msgs::msg::Detection2D();

      // Random bounding box center and size in image coordinates
      det.bbox.center.position.x = uniform_dist_(rng_) * 640.0;
      det.bbox.center.position.y = uniform_dist_(rng_) * 480.0;
      det.bbox.size_x = 50.0 + uniform_dist_(rng_) * 100.0;
      det.bbox.size_y = 50.0 + uniform_dist_(rng_) * 150.0;

      // Add hypothesis with confidence
      auto hypothesis = vision_msgs::msg::ObjectHypothesisWithPose();
      hypothesis.hypothesis.class_id = class_labels[class_dist(rng_)];
      hypothesis.hypothesis.score = 0.6 + uniform_dist_(rng_) * 0.4;  // 0.6 - 1.0
      det.results.push_back(hypothesis);

      det_array.detections.push_back(det);
    }

    // Inject false positive with probability
    if (uniform_dist_(rng_) < false_positive_rate_) {
      auto fp_det = vision_msgs::msg::Detection2D();
      fp_det.bbox.center.position.x = uniform_dist_(rng_) * 640.0;
      fp_det.bbox.center.position.y = uniform_dist_(rng_) * 480.0;
      fp_det.bbox.size_x = 30.0 + uniform_dist_(rng_) * 60.0;
      fp_det.bbox.size_y = 30.0 + uniform_dist_(rng_) * 60.0;

      auto fp_hypothesis = vision_msgs::msg::ObjectHypothesisWithPose();
      fp_hypothesis.hypothesis.class_id = "phantom";
      fp_hypothesis.hypothesis.score = 0.3 + uniform_dist_(rng_) * 0.3;  // low confidence
      fp_det.results.push_back(fp_hypothesis);

      det_array.detections.push_back(fp_det);
      false_positive_count_++;
    }

    detection_pub_->publish(det_array);

    // Publish diagnostics
    if (!has_input) {
      publish_diagnostics(
        "NO_INPUT",
        "No recent point cloud input - detections may be stale");
    } else if (false_positive_rate_ > 0.3) {
      publish_diagnostics(
        "HIGH_FP_RATE",
        "High false positive rate: " + std::to_string(false_positive_rate_));
    } else {
      publish_diagnostics(
        "OK",
        "Published " + std::to_string(det_array.detections.size()) + " detections");
    }
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "object_detector";
    diag.hardware_id = "perception_detector";

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

    kv.key = "msg_count";
    kv.value = std::to_string(msg_count_);
    diag.values.push_back(kv);

    kv.key = "false_positive_count";
    kv.value = std::to_string(false_positive_count_);
    diag.values.push_back(kv);

    kv.key = "last_cloud_points";
    kv.value = std::to_string(last_cloud_points_);
    diag.values.push_back(kv);

    kv.key = "false_positive_rate";
    kv.value = std::to_string(false_positive_rate_);
    diag.values.push_back(kv);

    kv.key = "miss_rate";
    kv.value = std::to_string(miss_rate_);
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // Publishers
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Parameters
  double false_positive_rate_;
  double miss_rate_;
  double failure_probability_;
  double detection_rate_;

  // State tracking
  rclcpp::Time last_cloud_time_{0, 0, RCL_SYSTEM_TIME};
  uint32_t last_cloud_points_{0};

  // Statistics
  uint64_t msg_count_{0};
  uint64_t false_positive_count_{0};
};

}  // namespace multi_ecu_demo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_ecu_demo::ObjectDetector>());
  rclcpp::shutdown();
  return 0;
}
