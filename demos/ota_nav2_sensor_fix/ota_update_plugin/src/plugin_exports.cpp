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

#include "ros2_medkit_gateway/plugins/plugin_types.hpp"

#include "ota_update_plugin/ota_update_plugin.hpp"

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return ros2_medkit_gateway::PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT ros2_medkit_gateway::GatewayPlugin * create_plugin() {
  return new ota_update_plugin::OtaUpdatePlugin();
}

// Explicit cross-cast so the gateway's plugin_loader can resolve the
// UpdateProvider interface without relying on dynamic_cast across the
// dlopen boundary (which is fragile when typeinfo isn't shared).
extern "C" GATEWAY_PLUGIN_EXPORT ros2_medkit_gateway::UpdateProvider *
get_update_provider(ros2_medkit_gateway::GatewayPlugin * plugin) {
  return dynamic_cast<ota_update_plugin::OtaUpdatePlugin *>(plugin);
}
