// Copyright 2026 selfpatch
// SPDX-License-Identifier: Apache-2.0

#include "beacon_helper.hpp"

#include <unistd.h>

#include "diagnostic_msgs/msg/key_value.hpp"

namespace sensor_diagnostics
{

BeaconHelper::BeaconHelper(rclcpp::Node * node, const Config & config)
: node_(node), config_(config)
{
  node_->declare_parameter("beacon_mode", "none");
  mode_ = node_->get_parameter("beacon_mode").as_string();

  // Cache process info (constant during lifetime)
  pid_ = static_cast<uint32_t>(getpid());
  char hostname_buf[256];
  hostname_buf[sizeof(hostname_buf) - 1] = '\0';
  if (gethostname(hostname_buf, sizeof(hostname_buf) - 1) == 0) {
    hostname_ = hostname_buf;
  }

  if (mode_ == "topic") {
    init_topic_beacon();
  } else if (mode_ == "param") {
    init_param_beacon();
  } else if (mode_ != "none") {
    RCLCPP_WARN(
      node_->get_logger(),
      "Unknown beacon_mode '%s', expected none/topic/param. Beacon disabled.",
      mode_.c_str());
  }
}

void BeaconHelper::init_topic_beacon()
{
  beacon_pub_ = node_->create_publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>(
    "/ros2_medkit/discovery", 10);

  beacon_timer_ = node_->create_wall_timer(
    std::chrono::seconds(5),
    [this]() { publish_beacon(); });

  // First publish likely dropped due to DDS discovery delay, next arrives in 5s
  publish_beacon();

  RCLCPP_INFO(
    node_->get_logger(), "Topic beacon enabled (entity: %s)",
    config_.entity_id.c_str());
}

void BeaconHelper::init_param_beacon()
{
  node_->declare_parameter("ros2_medkit.discovery.entity_id", config_.entity_id);
  node_->declare_parameter("ros2_medkit.discovery.display_name", config_.display_name);
  node_->declare_parameter("ros2_medkit.discovery.component_id", config_.component_id);
  node_->declare_parameter("ros2_medkit.discovery.function_ids", config_.function_ids);
  node_->declare_parameter(
    "ros2_medkit.discovery.process_id", static_cast<int64_t>(pid_));
  node_->declare_parameter("ros2_medkit.discovery.process_name", node_->get_name());

  if (!hostname_.empty()) {
    node_->declare_parameter("ros2_medkit.discovery.hostname", hostname_);
  }

  for (const auto & [key, value] : config_.metadata) {
    node_->declare_parameter("ros2_medkit.discovery.metadata." + key, value);
  }

  RCLCPP_INFO(
    node_->get_logger(), "Parameter beacon enabled (entity: %s)",
    config_.entity_id.c_str());
}

void BeaconHelper::publish_beacon()
{
  auto msg = ros2_medkit_msgs::msg::MedkitDiscoveryHint();
  msg.entity_id = config_.entity_id;
  msg.display_name = config_.display_name;
  msg.component_id = config_.component_id;
  msg.function_ids = config_.function_ids;
  msg.process_id = pid_;
  msg.process_name = node_->get_name();
  msg.hostname = hostname_;
  msg.stamp = node_->now();

  for (const auto & [key, value] : config_.metadata) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    msg.metadata.push_back(kv);
  }

  beacon_pub_->publish(msg);
}

}  // namespace sensor_diagnostics
