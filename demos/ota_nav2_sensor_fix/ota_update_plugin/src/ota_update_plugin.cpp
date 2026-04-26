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

#include "ota_update_plugin/ota_update_plugin.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <utility>

#include "catalog_client.hpp"
#include "operation_dispatcher.hpp"
#include "process_runner.hpp"

namespace ota_update_plugin {

namespace fs = std::filesystem;
using ros2_medkit_gateway::UpdateBackendError;
using ros2_medkit_gateway::UpdateBackendErrorInfo;

namespace {

/// Extract a packed tarball into a staging directory, then atomically replace
/// `${install_dir}/${target_package}` with the freshly extracted contents.
/// The artifacts are produced by pack_artifact.py and contain a single
/// top-level directory named after the target package.
tl::expected<void, std::string> extract_and_swap(const std::string & staged_tarball, const std::string & install_dir,
                                                 const std::string & target_package) {
  if (target_package.empty()) {
    return tl::make_unexpected("target_package is empty");
  }
  const std::string staging_extracted = staged_tarball + ".extracted";
  std::error_code ec;
  fs::remove_all(staging_extracted, ec);
  fs::create_directories(staging_extracted, ec);

  const std::string cmd = "tar -xzf " + staged_tarball + " -C " + staging_extracted;
  if (std::system(cmd.c_str()) != 0) {
    return tl::make_unexpected("tar extraction failed: " + cmd);
  }

  const std::string source = staging_extracted + "/" + target_package;
  if (!fs::exists(source)) {
    return tl::make_unexpected("artifact missing top-level directory '" + target_package + "' after extraction");
  }

  fs::create_directories(install_dir, ec);
  const std::string target = install_dir + "/" + target_package;
  fs::remove_all(target, ec);
  fs::copy(source, target, fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
  if (ec) {
    return tl::make_unexpected("copy failed: " + ec.message());
  }
  return {};
}

}  // namespace

OtaUpdatePlugin::OtaUpdatePlugin() : process_runner_(std::make_unique<ProcessRunner>()) {
}

OtaUpdatePlugin::~OtaUpdatePlugin() = default;

void OtaUpdatePlugin::configure(const nlohmann::json & config) {
  catalog_url_ = config.value("catalog_url", "http://ota_update_server:9000");
  staging_dir_ = config.value("staging_dir", "/tmp/ota_staging");
  install_dir_ = config.value("install_dir", "/ws/install");
  if (!catalog_client_) {
    catalog_client_ = std::make_unique<CatalogClient>(catalog_url_);
  }
}

void OtaUpdatePlugin::set_context(ros2_medkit_gateway::PluginContext & /*context*/) {
  poll_and_register_catalog();
}

void OtaUpdatePlugin::poll_and_register_catalog() {
  auto fetched = catalog_client_->fetch_catalog();
  if (!fetched) {
    std::fprintf(stderr, "[ota_update_plugin] catalog fetch failed: %s\n", fetched.error().c_str());
    return;
  }
  if (!fetched->is_array()) {
    std::fprintf(stderr, "[ota_update_plugin] catalog payload is not an array\n");
    return;
  }
  for (const auto & entry : *fetched) {
    auto rc = register_update(entry);
    if (!rc) {
      const std::string id = entry.value("id", "?");
      std::fprintf(stderr, "[ota_update_plugin] register %s failed: %s\n", id.c_str(), rc.error().message.c_str());
    }
  }
}

void OtaUpdatePlugin::set_catalog_client_for_test(std::unique_ptr<CatalogClient> client) {
  catalog_client_ = std::move(client);
}

void OtaUpdatePlugin::set_process_runner_for_test(std::unique_ptr<ProcessRunner> runner) {
  process_runner_ = std::move(runner);
}

tl::expected<std::vector<std::string>, UpdateBackendErrorInfo> OtaUpdatePlugin::list_updates(
    const ros2_medkit_gateway::UpdateFilter & /*filter*/) {
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<std::string> ids;
  ids.reserve(registry_.size());
  for (const auto & kv : registry_) {
    ids.push_back(kv.first);
  }
  return ids;
}

tl::expected<nlohmann::json, UpdateBackendErrorInfo> OtaUpdatePlugin::get_update(const std::string & id) {
  std::lock_guard<std::mutex> lk(mu_);
  auto it = registry_.find(id);
  if (it == registry_.end()) {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "update not registered"});
  }
  return it->second;
}

tl::expected<void, UpdateBackendErrorInfo> OtaUpdatePlugin::register_update(const nlohmann::json & metadata) {
  if (!metadata.contains("id") || !metadata["id"].is_string()) {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "metadata missing id"});
  }
  std::lock_guard<std::mutex> lk(mu_);
  registry_[metadata["id"].get<std::string>()] = metadata;
  return {};
}

tl::expected<void, UpdateBackendErrorInfo> OtaUpdatePlugin::delete_update(const std::string & id) {
  std::lock_guard<std::mutex> lk(mu_);
  registry_.erase(id);
  staged_artifacts_.erase(id);
  return {};
}

tl::expected<void, UpdateBackendErrorInfo> OtaUpdatePlugin::prepare(
    const std::string & id, ros2_medkit_gateway::UpdateProgressReporter & reporter) {
  nlohmann::json metadata;
  {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = registry_.find(id);
    if (it == registry_.end()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "no such update"});
    }
    metadata = it->second;
  }

  const auto kind = OperationDispatcher::classify(metadata);
  if (kind == OperationKind::Unknown) {
    return tl::make_unexpected(UpdateBackendErrorInfo{
        UpdateBackendError::InvalidInput,
        "update package must populate exactly one of "
        "updated_components / added_components / removed_components"});
  }

  if (kind == OperationKind::Uninstall) {
    reporter.set_progress(100);
    return {};
  }

  if (!metadata.contains("x_medkit_artifact_url") || !metadata["x_medkit_artifact_url"].is_string()) {
    return tl::make_unexpected(
        UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "missing x_medkit_artifact_url"});
  }

  std::error_code ec;
  fs::create_directories(staging_dir_, ec);
  const std::string url = metadata["x_medkit_artifact_url"].get<std::string>();
  const std::string staged_path = staging_dir_ + "/" + id + ".tar.gz";

  reporter.set_progress(10);
  auto dl = catalog_client_->download_artifact(url, staged_path);
  if (!dl) {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "download failed: " + dl.error()});
  }
  reporter.set_progress(80);

  {
    std::lock_guard<std::mutex> lk(mu_);
    staged_artifacts_[id] = *dl;
  }
  reporter.set_progress(100);
  return {};
}

tl::expected<void, UpdateBackendErrorInfo> OtaUpdatePlugin::execute(
    const std::string & id, ros2_medkit_gateway::UpdateProgressReporter & reporter) {
  nlohmann::json metadata;
  std::string staged;
  {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = registry_.find(id);
    if (it == registry_.end()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "no such update"});
    }
    metadata = it->second;
    auto sit = staged_artifacts_.find(id);
    staged = (sit != staged_artifacts_.end()) ? sit->second : "";
  }

  const auto kind = OperationDispatcher::classify(metadata);
  const std::string target_package = metadata.value("x_medkit_target_package", "");
  const std::string executable = metadata.value("x_medkit_executable", "");

  if (kind == OperationKind::Update) {
    if (staged.empty()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "call prepare() first"});
    }
    if (executable.empty()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "missing x_medkit_executable"});
    }
    // For an update across packages (e.g. broken_lidar -> fixed_lidar) the
    // OLD process binary lives in a different package than the NEW one we
    // are about to spawn, so its basename differs from `executable`. Honor
    // x_medkit_replaces_executable when present, fall back to executable.
    const std::string kill_target = metadata.value("x_medkit_replaces_executable", executable);
    reporter.set_progress(20);
    auto kr = process_runner_->kill_by_executable(kill_target);
    if (!kr) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "kill failed: " + kr.error()});
    }
    reporter.set_progress(40);
    if (auto sw = extract_and_swap(staged, install_dir_, target_package); !sw) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "swap failed: " + sw.error()});
    }
    reporter.set_progress(70);
    const std::string bin = install_dir_ + "/" + target_package + "/lib/" + target_package + "/" + executable;
    auto sp = process_runner_->spawn(bin);
    if (!sp) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "spawn failed: " + sp.error()});
    }
    reporter.set_progress(100);
    return {};
  }

  if (kind == OperationKind::Install) {
    if (staged.empty()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "call prepare() first"});
    }
    if (executable.empty()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "missing x_medkit_executable"});
    }
    reporter.set_progress(30);
    if (auto sw = extract_and_swap(staged, install_dir_, target_package); !sw) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "swap failed: " + sw.error()});
    }
    reporter.set_progress(70);
    const std::string bin = install_dir_ + "/" + target_package + "/lib/" + target_package + "/" + executable;
    auto sp = process_runner_->spawn(bin);
    if (!sp) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "spawn failed: " + sp.error()});
    }
    reporter.set_progress(100);
    return {};
  }

  if (kind == OperationKind::Uninstall) {
    reporter.set_progress(30);
    if (!target_package.empty()) {
      // Best-effort kill: legacy nodes may use the package name as their executable basename.
      // Failures are tolerated since the process may already be gone.
      auto kr = process_runner_->kill_by_executable(target_package);
      (void)kr;
      reporter.set_progress(70);
      std::error_code ec;
      fs::remove_all(install_dir_ + "/" + target_package, ec);
    }
    reporter.set_progress(100);
    return {};
  }

  return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "unknown operation kind"});
}

tl::expected<bool, UpdateBackendErrorInfo> OtaUpdatePlugin::supports_automated(const std::string & /*id*/) {
  return false;
}

}  // namespace ota_update_plugin
