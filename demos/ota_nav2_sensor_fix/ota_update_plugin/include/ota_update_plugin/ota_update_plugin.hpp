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
#include <ros2_medkit_gateway/core/plugins/gateway_plugin.hpp>
#include <ros2_medkit_gateway/core/providers/update_provider.hpp>
#include <ros2_medkit_gateway/dto/updates.hpp>

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
  tl::expected<ros2_medkit_gateway::dto::UpdateDetail, ros2_medkit_gateway::UpdateBackendErrorInfo> get_update(
      const std::string & id) override;
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
  // log and never attaches to the manifest entity tree. The fragment
  // filename keys on the target component (added/removed_components[0]),
  // shared by install + uninstall, so uninstall removes the same file.
  tl::expected<void, std::string> write_install_fragment(const std::string & update_id,
                                                          const nlohmann::json & metadata);
  tl::expected<void, std::string> remove_install_fragment(const nlohmann::json & metadata);
  void notify_manifest_changed();

  /// A process this plugin itself spawned, tracked so a later re-apply or
  /// uninstall can kill exactly that process (by recorded pid, pid-reuse
  /// guarded) instead of guessing at a catalog-supplied basename.
  struct SpawnedProc {
    int pid{-1};
    std::string executable;
  };

  /// If we spawned a process for `component`, terminate it (by recorded pid)
  /// and drop the record. No-op if we never spawned one for that component.
  void kill_previous_spawn(const std::string & component);

  std::string catalog_url_;
  std::string staging_dir_;
  std::string install_dir_;
  std::string fragments_dir_;

  ros2_medkit_gateway::PluginContext * context_{nullptr};

  std::mutex mu_;
  std::map<std::string, nlohmann::json> registry_;
  std::map<std::string, std::string> staged_artifacts_;
  // Keyed by target component: the process THIS plugin spawned for it.
  std::map<std::string, SpawnedProc> spawned_;

  std::unique_ptr<CatalogClient> catalog_client_;
  std::unique_ptr<ProcessRunner> process_runner_;
};

namespace detail {

/// Escape a string for emission as a double-quoted YAML scalar. Backslash and
/// double-quote are backslash-escaped; control characters (newline, CR, tab)
/// are emitted as YAML escapes so a value can never break out of its scalar or
/// inject a sibling key. Exposed for direct testing of the quoting behavior.
std::string yaml_quote(const std::string & value);

/// Render the manifest-fragment YAML body for one OTA-installed app, emitting
/// app_id / node_name / description as quoted scalars. Exposed for testing.
std::string render_install_fragment(const std::string & app_id, const std::string & node_name,
                                    const std::string & description);

}  // namespace detail

}  // namespace ota_update_plugin
