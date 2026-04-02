// Copyright 2026 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file point_cloud_filter.cpp
/// @brief Subscribes to LaserScan, converts to PointCloud2, and publishes filtered points
///
/// Converts incoming LaserScan ranges to XYZ points in a PointCloud2 message.
/// Supports point drop rate, artificial processing delay, and complete failure
/// injection. Diagnostics are published to /diagnostics.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace multi_ecu_demo
{

class PointCloudFilter : public rclcpp::Node
{
public:
  PointCloudFilter()
  : Node("point_cloud_filter"),
    rng_(std::random_device{}()),
    uniform_dist_(0.0, 1.0)
  {
    // Declare parameters with defaults
    this->declare_parameter("drop_rate", 0.0);            // 0.0 - 1.0 probability of dropping each point
    this->declare_parameter("delay_ms", 0);               // artificial processing delay in milliseconds
    this->declare_parameter("failure_probability", 0.0);  // 0.0 - 1.0

    load_parameters();

    // Create subscription to LaserScan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&PointCloudFilter::scan_callback, this, std::placeholders::_1));

    // Create publisher for filtered PointCloud2
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PointCloudFilter::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Point cloud filter started");
  }

private:
  void load_parameters()
  {
    drop_rate_ = this->get_parameter("drop_rate").as_double();
    delay_ms_ = this->get_parameter("delay_ms").as_int();
    failure_probability_ = this->get_parameter("failure_probability").as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "drop_rate") {
        drop_rate_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Drop rate changed to %.2f", drop_rate_);
      } else if (param.get_name() == "delay_ms") {
        delay_ms_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Delay changed to %d ms", delay_ms_);
      } else if (param.get_name() == "failure_probability") {
        failure_probability_ = param.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Failure probability changed to %.2f",
          failure_probability_);
      }
    }

    return result;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    msg_count_++;

    // Intentional blocking sleep to simulate slow processing pipeline.
    // This blocks the single-threaded executor during the delay.
    if (delay_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms_));
    }

    // Check for complete failure - publish empty cloud
    if (uniform_dist_(rng_) < failure_probability_) {
      publish_empty_cloud(scan->header);
      publish_diagnostics("FAILURE", "Filter failure (injected) - empty cloud published");
      return;
    }

    // Convert LaserScan to PointCloud2 with filtering
    std::vector<float> points;  // x, y, z triples
    int input_count = static_cast<int>(scan->ranges.size());
    int dropped_count = 0;

    for (size_t i = 0; i < scan->ranges.size(); i++) {
      float range = scan->ranges[i];

      // Skip invalid ranges
      if (std::isnan(range) || std::isinf(range) ||
        range < scan->range_min || range > scan->range_max)
      {
        dropped_count++;
        continue;
      }

      // Apply drop rate - probability of dropping each point
      if (uniform_dist_(rng_) < drop_rate_) {
        dropped_count++;
        continue;
      }

      // Convert polar to Cartesian (x, y, z=0 for 2D scan)
      float angle = scan->angle_min + static_cast<float>(i) * scan->angle_increment;
      float x = range * std::cos(angle);
      float y = range * std::sin(angle);
      float z = 0.0f;

      points.push_back(x);
      points.push_back(y);
      points.push_back(z);
    }

    // Build PointCloud2 message
    auto cloud = sensor_msgs::msg::PointCloud2();
    cloud.header = scan->header;
    cloud.header.frame_id = "lidar_link";

    uint32_t num_points = static_cast<uint32_t>(points.size() / 3);
    cloud.height = 1;
    cloud.width = num_points;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    // Define point fields: x, y, z (float32 each)
    sensor_msgs::msg::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;

    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;

    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    cloud.fields = {field_x, field_y, field_z};
    cloud.point_step = 12;  // 3 floats * 4 bytes
    cloud.row_step = cloud.point_step * num_points;

    // Copy point data
    cloud.data.resize(points.size() * sizeof(float));
    std::memcpy(cloud.data.data(), points.data(), cloud.data.size());

    cloud_pub_->publish(cloud);

    // Publish diagnostics
    int output_count = static_cast<int>(num_points);
    if (drop_rate_ > 0.5) {
      publish_diagnostics(
        "HIGH_DROP_RATE",
        "Dropping " + std::to_string(dropped_count) + "/" + std::to_string(input_count) +
        " points");
    } else if (delay_ms_ > 100) {
      publish_diagnostics(
        "HIGH_LATENCY",
        "Processing delay: " + std::to_string(delay_ms_) + "ms");
    } else {
      publish_diagnostics(
        "OK",
        "Filtered " + std::to_string(input_count) + " -> " + std::to_string(output_count) +
        " points");
    }
  }

  void publish_empty_cloud(const std_msgs::msg::Header & header)
  {
    auto cloud = sensor_msgs::msg::PointCloud2();
    cloud.header = header;
    cloud.header.frame_id = "lidar_link";
    cloud.height = 1;
    cloud.width = 0;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::msg::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;

    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;

    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    cloud.fields = {field_x, field_y, field_z};
    cloud.point_step = 12;
    cloud.row_step = 0;

    cloud_pub_->publish(cloud);
  }

  void publish_diagnostics(const std::string & status, const std::string & message)
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();

    auto diag = diagnostic_msgs::msg::DiagnosticStatus();
    diag.name = "point_cloud_filter";
    diag.hardware_id = "perception_filter";

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

    kv.key = "drop_rate";
    kv.value = std::to_string(drop_rate_);
    diag.values.push_back(kv);

    kv.key = "delay_ms";
    kv.value = std::to_string(delay_ms_);
    diag.values.push_back(kv);

    kv.key = "failure_probability";
    kv.value = std::to_string(failure_probability_);
    diag.values.push_back(kv);

    diag_array.status.push_back(diag);
    diag_pub_->publish(diag_array);
  }

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Parameters
  double drop_rate_;
  int delay_ms_;
  double failure_probability_;

  // Statistics
  uint64_t msg_count_{0};
};

}  // namespace multi_ecu_demo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_ecu_demo::PointCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
