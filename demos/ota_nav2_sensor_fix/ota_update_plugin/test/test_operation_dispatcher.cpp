// Copyright 2026 bburda
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "operation_dispatcher.hpp"

using ota_update_plugin::OperationDispatcher;
using ota_update_plugin::OperationKind;

TEST(OperationDispatcher, UpdateFromUpdatedComponents) {
  nlohmann::json m = {{"id", "x"}, {"updated_components", {"scan_sensor_node"}}};
  EXPECT_EQ(OperationDispatcher::classify(m), OperationKind::Update);
}

TEST(OperationDispatcher, InstallFromAddedComponents) {
  nlohmann::json m = {{"id", "x"}, {"added_components", {"obstacle_classifier"}}};
  EXPECT_EQ(OperationDispatcher::classify(m), OperationKind::Install);
}

TEST(OperationDispatcher, UninstallFromRemovedComponents) {
  nlohmann::json m = {{"id", "x"}, {"removed_components", {"broken_lidar_legacy"}}};
  EXPECT_EQ(OperationDispatcher::classify(m), OperationKind::Uninstall);
}

TEST(OperationDispatcher, UnknownWhenAllEmpty) {
  nlohmann::json m = {{"id", "x"}};
  EXPECT_EQ(OperationDispatcher::classify(m), OperationKind::Unknown);
}

TEST(OperationDispatcher, UnknownWhenMixed) {
  nlohmann::json m = {
      {"id", "x"},
      {"added_components", {"a"}},
      {"removed_components", {"b"}},
  };
  EXPECT_EQ(OperationDispatcher::classify(m), OperationKind::Unknown);
}

TEST(OperationDispatcher, UnknownWhenComponentsAreEmptyArray) {
  nlohmann::json m = {{"id", "x"}, {"updated_components", nlohmann::json::array()}};
  EXPECT_EQ(OperationDispatcher::classify(m), OperationKind::Unknown);
}

TEST(OperationDispatcher, UnknownWhenComponentsIsNotArray) {
  nlohmann::json m = {{"id", "x"}, {"updated_components", "scan_sensor_node"}};
  EXPECT_EQ(OperationDispatcher::classify(m), OperationKind::Unknown);
}
