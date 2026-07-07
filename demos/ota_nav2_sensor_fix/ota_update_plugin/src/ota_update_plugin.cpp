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

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <utility>
#include <vector>

#include <ros2_medkit_gateway/core/plugins/entity_change_scope.hpp>
#include <ros2_medkit_gateway/core/plugins/plugin_context.hpp>
#include <ros2_medkit_gateway/dto/updates.hpp>

#include "catalog_client.hpp"
#include "operation_dispatcher.hpp"
#include "process_runner.hpp"

namespace ota_update_plugin {

namespace fs = std::filesystem;
using ros2_medkit_gateway::UpdateBackendError;
using ros2_medkit_gateway::UpdateBackendErrorInfo;

namespace {

/// Reject any catalog-supplied name (component / update id / executable /
/// target package) that could escape the staging or install tree, or inject a
/// process/tar argument. Only `[A-Za-z0-9._-]` is allowed; empty, "." and ".."
/// are rejected explicitly (a no-op or a parent-dir escape as a path segment).
/// '/' and whitespace are outside the allow-list, so path separators and
/// argument injection are rejected as a consequence.
bool is_safe_component(const std::string & s) {
  if (s.empty() || s == "." || s == "..") {
    return false;
  }
  for (const char c : s) {
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '.' ||
                    c == '_' || c == '-';
    if (!ok) {
      return false;
    }
  }
  return true;
}

/// First string element of a SOVD component array (updated / added / removed),
/// or "" if the key is absent, empty, or the element is not a string.
std::string first_component(const nlohmann::json & metadata, const char * key) {
  if (!metadata.contains(key) || !metadata[key].is_array() || metadata[key].empty()) {
    return {};
  }
  const auto & first = metadata[key][0];
  return first.is_string() ? first.get<std::string>() : std::string{};
}

/// Run `tar -xzf <staged_tarball> -C <dest_dir>` via fork + execvp - no shell,
/// so a catalog-controlled name can never be interpreted as a tar option or a
/// shell metacharacter. Waits for tar and maps a non-zero exit to an error.
tl::expected<void, std::string> run_tar_extract(const std::string & staged_tarball, const std::string & dest_dir) {
  // execvp wants `char* const[]`. std::string::data() is non-const since C++17
  // and execvp does not modify the buffers, so no copy or const_cast is needed.
  std::vector<std::string> args = {"tar", "-xzf", staged_tarball, "-C", dest_dir};
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (auto & a : args) {
    argv.push_back(a.data());
  }
  argv.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    return tl::make_unexpected(std::string("fork failed: ") + std::strerror(errno));
  }
  if (pid == 0) {
    execvp(argv[0], argv.data());
    _exit(127);  // reached only if exec failed
  }
  int status = 0;
  if (::waitpid(pid, &status, 0) < 0) {
    return tl::make_unexpected(std::string("waitpid failed: ") + std::strerror(errno));
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    return tl::make_unexpected("tar extraction failed for " + staged_tarball);
  }
  return {};
}

/// Stage half of a stage-then-swap: extract the tarball into a scratch dir and
/// verify it carries the expected top-level package directory. Does NOT touch
/// the live install tree - the caller only swaps it into place after this
/// succeeds, so a corrupt tarball can never leave a half-removed binary behind
/// (the update path must not kill the running node before this returns).
/// Returns the path to the extracted package tree on success.
tl::expected<std::string, std::string> extract_to_staging(const std::string & staged_tarball,
                                                          const std::string & target_package) {
  const std::string staging_extracted = staged_tarball + ".extracted";
  std::error_code ec;
  fs::remove_all(staging_extracted, ec);
  fs::create_directories(staging_extracted, ec);

  if (auto ex = run_tar_extract(staged_tarball, staging_extracted); !ex) {
    return tl::make_unexpected(ex.error());
  }

  const std::string source = staging_extracted + "/" + target_package;
  if (!fs::exists(source)) {
    return tl::make_unexpected("artifact missing top-level directory '" + target_package + "' after extraction");
  }
  return source;
}

/// Swap half of a stage-then-swap: replace `${install_dir}/${target_package}`
/// with the freshly extracted tree. This is the destructive step and runs only
/// after extract_to_staging() succeeded. Not a true atomic rename (source and
/// target may sit on different filesystems), but the old tree is removed only
/// once a verified new tree exists, so there is no window with neither.
tl::expected<void, std::string> swap_into_place(const std::string & source, const std::string & install_dir,
                                                const std::string & target_package) {
  std::error_code ec;
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
  // Where this plugin drops manifest fragments for OTA-installed apps.
  // Must equal the path the gateway has configured under
  // discovery.manifest.fragments_dir, otherwise the gateway won't pick
  // them up on reload. Empty disables fragment writes (legacy behavior:
  // installed nodes appear as orphans in the entity tree).
  fragments_dir_ = config.value("fragments_dir", "");
  if (!catalog_client_) {
    catalog_client_ = std::make_unique<CatalogClient>(catalog_url_);
  }
}

void OtaUpdatePlugin::set_context(ros2_medkit_gateway::PluginContext & context) {
  // Hold on to the context so post-execute we can ask the gateway to
  // re-merge manifest fragments and rerun discovery via
  // notify_entities_changed.
  context_ = &context;
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

tl::expected<ros2_medkit_gateway::dto::UpdateDetail, UpdateBackendErrorInfo> OtaUpdatePlugin::get_update(
    const std::string & id) {
  std::lock_guard<std::mutex> lk(mu_);
  auto it = registry_.find(id);
  if (it == registry_.end()) {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "update not registered"});
  }
  return ros2_medkit_gateway::dto::UpdateDetail{it->second};
}

tl::expected<void, UpdateBackendErrorInfo> OtaUpdatePlugin::register_update(const nlohmann::json & metadata) {
  if (!metadata.contains("id") || !metadata["id"].is_string()) {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "metadata missing id"});
  }
  const std::string id = metadata["id"].get<std::string>();
  // The id becomes a staging path segment in prepare() (staging_dir_/<id>.tar.gz).
  // Reject an unsafe id here so it never enters the registry - a bad id can then
  // never reach a filesystem or exec path downstream.
  if (!is_safe_component(id)) {
    return tl::make_unexpected(
        UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "update id contains unsafe characters: " + id});
  }
  std::lock_guard<std::mutex> lk(mu_);
  registry_[id] = metadata;
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
    if (executable.empty()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "missing x_medkit_executable"});
    }
    // Validate every catalog-controlled name BEFORE it reaches a path or exec:
    // target_package builds the extract/install path and the spawn binary,
    // executable is the argv[0] we spawn.
    if (!is_safe_component(target_package)) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "x_medkit_target_package is unsafe"});
    }
    if (!is_safe_component(executable)) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "x_medkit_executable is unsafe"});
    }
    if (staged.empty()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "call prepare() first"});
    }
    const std::string component = first_component(metadata, "updated_components");

    // Stage-then-swap: extract + verify the new tree FIRST. A bad tarball
    // returns an error here without ever stopping the running node - the kill
    // below happens only once we hold a good extract.
    reporter.set_progress(20);
    auto src = extract_to_staging(staged, target_package);
    if (!src) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::Internal, "extract failed: " + src.error()});
    }
    reporter.set_progress(40);

    // The kill is ADDITIVE. At boot fixed_lidar_node is spawned by the launch
    // file, not this plugin, so the only handle we have on it is its basename
    // (x_medkit_replaces_executable, falling back to executable). ALSO kill any
    // node WE spawned earlier for this component so a re-apply of the same
    // update does not leave a duplicate.
    const std::string kill_target = metadata.value("x_medkit_replaces_executable", executable);
    auto kr = process_runner_->kill_by_executable(kill_target);
    if (!kr) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "kill failed: " + kr.error()});
    }
    kill_previous_spawn(component);

    if (auto sw = swap_into_place(*src, install_dir_, target_package); !sw) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "swap failed: " + sw.error()});
    }
    reporter.set_progress(70);
    const std::string bin = install_dir_ + "/" + target_package + "/lib/" + target_package + "/" + executable;
    auto sp = process_runner_->spawn(bin);
    if (!sp) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "spawn failed: " + sp.error()});
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      spawned_[component] = SpawnedProc{*sp, executable};
    }
    // Update flow: same app id (the binary swapped in is bound to the
    // same scan_sensor_node entity as the binary it replaced) - no
    // manifest fragment to write, but the gateway still needs to
    // rerun discovery so the new pid / process metadata replaces the
    // stale entries in the entity cache.
    notify_manifest_changed();
    reporter.set_progress(100);
    return {};
  }

  if (kind == OperationKind::Install) {
    if (executable.empty()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "missing x_medkit_executable"});
    }
    if (!is_safe_component(target_package)) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "x_medkit_target_package is unsafe"});
    }
    if (!is_safe_component(executable)) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "x_medkit_executable is unsafe"});
    }
    if (staged.empty()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "call prepare() first"});
    }
    reporter.set_progress(30);
    auto src = extract_to_staging(staged, target_package);
    if (!src) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::Internal, "extract failed: " + src.error()});
    }
    if (auto sw = swap_into_place(*src, install_dir_, target_package); !sw) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "swap failed: " + sw.error()});
    }
    reporter.set_progress(70);
    const std::string bin = install_dir_ + "/" + target_package + "/lib/" + target_package + "/" + executable;
    auto sp = process_runner_->spawn(bin);
    if (!sp) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "spawn failed: " + sp.error()});
    }
    const std::string component = first_component(metadata, "added_components");
    {
      std::lock_guard<std::mutex> lk(mu_);
      spawned_[component] = SpawnedProc{*sp, executable};
    }
    // Install flow: NEW app entity. Write a manifest fragment so the
    // gateway picks the new app up under the manifest tree (otherwise
    // it stays as an "Orphan node (not in manifest)" warn log and
    // never appears under the rbtheron component / Functions
    // listing). Notify even when fragment write fails - the spawn
    // already happened and discovery should still see the new node.
    if (auto fr = write_install_fragment(id, metadata); !fr) {
      std::fprintf(stderr, "[ota_update_plugin] fragment write failed for %s: %s\n", id.c_str(),
                   fr.error().c_str());
    }
    notify_manifest_changed();
    reporter.set_progress(100);
    return {};
  }

  if (kind == OperationKind::Uninstall) {
    if (!target_package.empty() && !is_safe_component(target_package)) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "x_medkit_target_package is unsafe"});
    }
    const std::string component = first_component(metadata, "removed_components");
    reporter.set_progress(30);
    // Kill the node WE spawned for this component. Its argv[0] is the recorded
    // executable basename, NOT target_package - the running node identifies as
    // the executable, so killing by the package name missed it and left an
    // orphan. No-op if we never spawned it (e.g. a base-manifest node).
    kill_previous_spawn(component);
    reporter.set_progress(70);
    if (!target_package.empty()) {
      std::error_code ec;
      fs::remove_all(install_dir_ + "/" + target_package, ec);
    }
    // Uninstall: drop the fragment we wrote at install time (keyed on the same
    // component) and rerun discovery so the entity tree no longer lists the
    // now-dead app. Entities defined in the base manifest stay - fragments only
    // ADD, they can't remove base-manifest declarations - those go offline.
    if (auto fr = remove_install_fragment(metadata); !fr) {
      std::fprintf(stderr, "[ota_update_plugin] fragment remove failed for %s: %s\n", id.c_str(),
                   fr.error().c_str());
    }
    notify_manifest_changed();
    reporter.set_progress(100);
    return {};
  }

  return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "unknown operation kind"});
}

tl::expected<bool, UpdateBackendErrorInfo> OtaUpdatePlugin::supports_automated(const std::string & /*id*/) {
  // This plugin applies updates itself on execute (kill + respawn the target
  // binary), so every update it serves supports automated application. The UI's
  // combined "Prepare & execute" action gates on this; returning false made it
  // report "package does not support automatic updates".
  return true;
}

namespace detail {

std::string yaml_quote(const std::string & value) {
  std::string out = "\"";
  for (const char c : value) {
    switch (c) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
        break;
    }
  }
  out += "\"";
  return out;
}

// Build the YAML body for a single OTA-installed app. app_id, node_name and
// description are catalog-derived, so they are emitted as double-quoted scalars
// via yaml_quote(): a space, ':' or newline in a value can neither break out of
// its scalar nor inject a sibling key. The base manifest defines the `rbtheron`
// component; fragments only ever add apps onto it.
std::string render_install_fragment(const std::string & app_id, const std::string & node_name,
                                    const std::string & description) {
  std::string out;
  out += "manifest_version: \"1.0\"\n";
  out += "apps:\n";
  out += "  - id: " + yaml_quote(app_id) + "\n";
  out += "    name: " + yaml_quote(app_id) + "\n";
  out += "    category: \"ota-installed\"\n";
  out += "    is_located_on: rbtheron\n";
  out += "    description: " + yaml_quote(description) + "\n";
  out += "    ros_binding: { node_name: " + yaml_quote(node_name) + ", namespace: / }\n";
  return out;
}

}  // namespace detail

tl::expected<void, std::string> OtaUpdatePlugin::write_install_fragment(const std::string & update_id,
                                                                         const nlohmann::json & metadata) {
  if (fragments_dir_.empty()) return {};

  const std::string node_name = metadata.value("x_medkit_executable", "");
  // SOVD ISO 17978-3 reports the target entity via `added_components`
  // (it's an array; for an OTA install we always have exactly one). The
  // component is the fragment's stable key: install writes <component>.yaml
  // and uninstall removes the same file.
  const std::string app_id = first_component(metadata, "added_components");
  if (node_name.empty() || app_id.empty()) {
    return tl::make_unexpected("metadata missing x_medkit_executable / added_components for fragment");
  }
  // The component becomes a path segment (the fragment filename), so validate
  // it before building the path.
  if (!is_safe_component(app_id)) {
    return tl::make_unexpected("added_components[0] contains unsafe characters: " + app_id);
  }
  const std::string description = "OTA-installed via " + update_id;

  std::error_code ec;
  fs::create_directories(fragments_dir_, ec);
  if (ec) {
    return tl::make_unexpected("create fragments_dir failed: " + ec.message());
  }

  const std::string final_path = fragments_dir_ + "/" + app_id + ".yaml";
  const std::string tmp_path = fragments_dir_ + "/.tmp-" + app_id + ".yaml";
  // Atomic publish per ManifestManager's fragment contract: write to
  // tmp, fsync, rename. The gateway's fragment scanner runs on the
  // notify_entities_changed thread - a half-written file would fail
  // the manifest reload and roll back the entire merge.
  {
    std::ofstream f(tmp_path, std::ios::binary | std::ios::trunc);
    if (!f) return tl::make_unexpected("open tmp fragment failed: " + tmp_path);
    f << detail::render_install_fragment(app_id, node_name, description);
    f.flush();
    if (!f) return tl::make_unexpected("write tmp fragment failed: " + tmp_path);
  }
  if (std::rename(tmp_path.c_str(), final_path.c_str()) != 0) {
    return tl::make_unexpected("rename fragment failed: " + std::string(std::strerror(errno)));
  }
  return {};
}

tl::expected<void, std::string> OtaUpdatePlugin::remove_install_fragment(const nlohmann::json & metadata) {
  if (fragments_dir_.empty()) return {};
  // Same key as write_install_fragment: the target component, reported via
  // removed_components for an uninstall.
  const std::string component = first_component(metadata, "removed_components");
  if (component.empty()) {
    return tl::make_unexpected("metadata missing removed_components for fragment removal");
  }
  if (!is_safe_component(component)) {
    return tl::make_unexpected("removed_components[0] contains unsafe characters: " + component);
  }
  std::error_code ec;
  fs::remove(fragments_dir_ + "/" + component + ".yaml", ec);
  // Missing-file is fine (uninstall of an entity that lived in the
  // base manifest, never had a fragment); other errors are reported.
  if (ec && ec != std::errc::no_such_file_or_directory) {
    return tl::make_unexpected("remove fragment failed: " + ec.message());
  }
  return {};
}

void OtaUpdatePlugin::kill_previous_spawn(const std::string & component) {
  SpawnedProc prev;
  {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = spawned_.find(component);
    if (it == spawned_.end()) {
      return;
    }
    prev = it->second;
    spawned_.erase(it);
  }
  // kill_pid is pid-reuse guarded: it only signals if argv[0] still matches the
  // executable we recorded, so a recycled pid is never harmed. Dropping the
  // record above is correct either way (the process is gone or no longer ours).
  process_runner_->kill_pid(prev.pid, prev.executable);
}

void OtaUpdatePlugin::notify_manifest_changed() {
  if (!context_) return;
  context_->notify_entities_changed(ros2_medkit_gateway::EntityChangeScope::full_refresh());
}

}  // namespace ota_update_plugin
