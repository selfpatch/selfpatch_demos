// Copyright 2026 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file beacon_helper.hpp
/// @brief Shared helper for beacon mode support in sensor demo nodes
///
/// Encapsulates topic beacon (push) and parameter beacon (pull) logic.
/// Each sensor node creates a BeaconHelper with its entity config.
/// The beacon_mode parameter ("none", "topic", "param") controls behavior.

#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

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
    std::map<std::string, std::string> metadata;
  };

  BeaconHelper(rclcpp::Node * node, const Config & config);

private:
  void init_topic_beacon();
  void init_param_beacon();
  void publish_beacon();

  rclcpp::Node * node_;
  Config config_;
  std::string mode_;
  uint32_t pid_{0};
  std::string hostname_;
  rclcpp::Publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>::SharedPtr beacon_pub_;
  rclcpp::TimerBase::SharedPtr beacon_timer_;
};

}  // namespace sensor_diagnostics
