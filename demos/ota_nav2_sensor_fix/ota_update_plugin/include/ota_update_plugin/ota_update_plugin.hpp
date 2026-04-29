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

#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <ros2_medkit_gateway/plugins/gateway_plugin.hpp>
// UpdateProvider lives at providers/ in newer gateway revisions and updates/ in older ones.
#if __has_include(<ros2_medkit_gateway/providers/update_provider.hpp>)
#include <ros2_medkit_gateway/providers/update_provider.hpp>
#else
#include <ros2_medkit_gateway/updates/update_provider.hpp>
#endif

namespace ota_update_plugin {

class CatalogClient;
class ProcessRunner;

/// OTA update plugin: implements both GatewayPlugin and UpdateProvider.
/// Polls a FastAPI catalog at boot and supports update / install / uninstall
/// operations derived from SOVD ISO 17978-3 metadata.
class OtaUpdatePlugin : public ros2_medkit_gateway::GatewayPlugin, public ros2_medkit_gateway::UpdateProvider {
 public:
  OtaUpdatePlugin();
  ~OtaUpdatePlugin() override;

  OtaUpdatePlugin(const OtaUpdatePlugin &) = delete;
  OtaUpdatePlugin & operator=(const OtaUpdatePlugin &) = delete;
  OtaUpdatePlugin(OtaUpdatePlugin &&) = delete;
  OtaUpdatePlugin & operator=(OtaUpdatePlugin &&) = delete;

  // GatewayPlugin
  std::string name() const override {
    return "ota_update_plugin";
  }
  void configure(const nlohmann::json & config) override;
  void set_context(ros2_medkit_gateway::PluginContext & context) override;

  // UpdateProvider
  tl::expected<std::vector<std::string>, ros2_medkit_gateway::UpdateBackendErrorInfo> list_updates(
      const ros2_medkit_gateway::UpdateFilter & filter) override;
  tl::expected<nlohmann::json, ros2_medkit_gateway::UpdateBackendErrorInfo> get_update(const std::string & id) override;
  tl::expected<void, ros2_medkit_gateway::UpdateBackendErrorInfo> register_update(
      const nlohmann::json & metadata) override;
  tl::expected<void, ros2_medkit_gateway::UpdateBackendErrorInfo> delete_update(const std::string & id) override;
  tl::expected<void, ros2_medkit_gateway::UpdateBackendErrorInfo> prepare(
      const std::string & id, ros2_medkit_gateway::UpdateProgressReporter & reporter) override;
  tl::expected<void, ros2_medkit_gateway::UpdateBackendErrorInfo> execute(
      const std::string & id, ros2_medkit_gateway::UpdateProgressReporter & reporter) override;
  tl::expected<bool, ros2_medkit_gateway::UpdateBackendErrorInfo> supports_automated(const std::string & id) override;

  // Test seams
  void set_catalog_client_for_test(std::unique_ptr<CatalogClient> client);
  void set_process_runner_for_test(std::unique_ptr<ProcessRunner> runner);
  void poll_and_register_catalog();

 private:
  // Manifest-fragment helpers. Plugins that deploy new nodes at runtime
  // are expected to drop a fragment yaml in `fragments_dir_` and then
  // notify the gateway so its ManifestManager re-merges. Without this
  // the new app shows up as an "Orphan node (not in manifest)" warn
  // log and never attaches to the manifest entity tree.
  tl::expected<void, std::string> write_install_fragment(const std::string & update_id,
                                                          const nlohmann::json & metadata);
  tl::expected<void, std::string> remove_install_fragment(const std::string & update_id);
  void notify_manifest_changed();

  std::string catalog_url_;
  std::string staging_dir_;
  std::string install_dir_;
  std::string fragments_dir_;

  ros2_medkit_gateway::PluginContext * context_{nullptr};

  std::mutex mu_;
  std::map<std::string, nlohmann::json> registry_;
  std::map<std::string, std::string> staged_artifacts_;

  std::unique_ptr<CatalogClient> catalog_client_;
  std::unique_ptr<ProcessRunner> process_runner_;
};

}  // namespace ota_update_plugin
