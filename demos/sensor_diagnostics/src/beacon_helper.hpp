// Copyright 2026 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file beacon_helper.hpp
/// @brief Shared helper for beacon mode support in sensor demo nodes
///
/// Encapsulates topic beacon (push) and parameter beacon (pull) logic.
/// Each sensor node creates a BeaconHelper with its entity config.
/// The beacon_mode parameter ("none", "topic", "param") controls behavior.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <unistd.h>

#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_msgs/msg/medkit_discovery_hint.hpp"

namespace sensor_diagnostics
{

class BeaconHelper
{
public:
  struct Config
  {
    std::string entity_id;
    std::string display_name;
    std::string component_id;
    std::vector<std::string> function_ids;
    std::unordered_map<std::string, std::string> metadata;
  };

  BeaconHelper(rclcpp::Node * node, const Config & config)
  : node_(node), config_(config)
  {
    node_->declare_parameter("beacon_mode", "none");
    mode_ = node_->get_parameter("beacon_mode").as_string();

    if (mode_ == "topic") {
      init_topic_beacon();
    } else if (mode_ == "param") {
      init_param_beacon();
    }
  }

private:
  void init_topic_beacon()
  {
    beacon_pub_ = node_->create_publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>(
      "/ros2_medkit/discovery", 10);

    beacon_timer_ = node_->create_wall_timer(
      std::chrono::seconds(5),
      [this]() { publish_beacon(); });

    // Publish immediately on startup
    publish_beacon();

    RCLCPP_INFO(
      node_->get_logger(), "Topic beacon enabled (entity: %s)",
      config_.entity_id.c_str());
  }

  void init_param_beacon()
  {
    node_->declare_parameter("ros2_medkit.discovery.entity_id", config_.entity_id);
    node_->declare_parameter("ros2_medkit.discovery.display_name", config_.display_name);
    node_->declare_parameter("ros2_medkit.discovery.component_id", config_.component_id);
    node_->declare_parameter("ros2_medkit.discovery.function_ids", config_.function_ids);
    node_->declare_parameter(
      "ros2_medkit.discovery.process_id", static_cast<int>(getpid()));
    node_->declare_parameter("ros2_medkit.discovery.process_name", node_->get_name());

    char hostname_buf[256];
    if (gethostname(hostname_buf, sizeof(hostname_buf)) == 0) {
      node_->declare_parameter("ros2_medkit.discovery.hostname", std::string(hostname_buf));
    }

    for (const auto & [key, value] : config_.metadata) {
      node_->declare_parameter("ros2_medkit.discovery.metadata." + key, value);
    }

    RCLCPP_INFO(
      node_->get_logger(), "Parameter beacon enabled (entity: %s)",
      config_.entity_id.c_str());
  }

  void publish_beacon()
  {
    auto msg = ros2_medkit_msgs::msg::MedkitDiscoveryHint();
    msg.entity_id = config_.entity_id;
    msg.display_name = config_.display_name;
    msg.component_id = config_.component_id;
    msg.function_ids = config_.function_ids;
    msg.process_id = static_cast<uint32_t>(getpid());
    msg.process_name = node_->get_name();
    msg.stamp = node_->now();

    char hostname_buf[256];
    if (gethostname(hostname_buf, sizeof(hostname_buf)) == 0) {
      msg.hostname = hostname_buf;
    }

    for (const auto & [key, value] : config_.metadata) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      msg.metadata.push_back(kv);
    }

    beacon_pub_->publish(msg);
  }

  rclcpp::Node * node_;
  Config config_;
  std::string mode_;
  rclcpp::Publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>::SharedPtr beacon_pub_;
  rclcpp::TimerBase::SharedPtr beacon_timer_;
};

}  // namespace sensor_diagnostics
