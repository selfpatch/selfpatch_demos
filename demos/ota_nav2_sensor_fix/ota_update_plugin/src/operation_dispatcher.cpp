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

#include "operation_dispatcher.hpp"

namespace ota_update_plugin {

namespace {

bool non_empty_array(const nlohmann::json & j, const char * key) {
  if (!j.contains(key)) {
    return false;
  }
  const auto & v = j.at(key);
  return v.is_array() && !v.empty();
}

}  // namespace

OperationKind OperationDispatcher::classify(const nlohmann::json & metadata) {
  const bool has_updated = non_empty_array(metadata, "updated_components");
  const bool has_added = non_empty_array(metadata, "added_components");
  const bool has_removed = non_empty_array(metadata, "removed_components");
  const int populated = static_cast<int>(has_updated) + static_cast<int>(has_added) + static_cast<int>(has_removed);
  if (populated != 1) {
    return OperationKind::Unknown;
  }
  if (has_updated) {
    return OperationKind::Update;
  }
  if (has_added) {
    return OperationKind::Install;
  }
  return OperationKind::Uninstall;
}

}  // namespace ota_update_plugin
