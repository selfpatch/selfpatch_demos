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

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <ros2_medkit_gateway/core/providers/update_types.hpp>

#include "catalog_client.hpp"
#include "ota_update_plugin/ota_update_plugin.hpp"
#include "process_runner.hpp"

namespace {

class FakeCatalogClient : public ota_update_plugin::CatalogClient {
 public:
  using CatalogClient::CatalogClient;

  nlohmann::json catalog_payload = nlohmann::json::array();
  std::string artifact_to_return = "TARDATA";
  std::string requested_url;

  tl::expected<nlohmann::json, std::string> fetch_catalog() override {
    return catalog_payload;
  }

  tl::expected<std::string, std::string> download_artifact(const std::string & url, const std::string & out) override {
    requested_url = url;
    std::ofstream o(out, std::ios::binary);
    o << artifact_to_return;  // binary-safe: writes embedded NULs of real gzip
    return out;
  }
};

/// Stateful ProcessRunner double modelling a tiny process table so tests can
/// assert exactly which processes are live after a sequence of spawn/kill
/// calls. spawn() hands out monotonically increasing fake pids; the kill
/// methods remove matching entries. No real process is ever created.
class FakeProcessRunner : public ota_update_plugin::ProcessRunner {
 public:
  struct Entry {
    int pid;
    std::string executable;
  };

  std::string last_kill_target;

  std::vector<int> pgrep(const std::string & executable_basename) override {
    std::vector<int> out;
    for (const auto & e : table_) {
      if (e.executable == executable_basename) {
        out.push_back(e.pid);
      }
    }
    return out;
  }

  tl::expected<int, std::string> kill_by_executable(const std::string & executable_basename,
                                                    int /*timeout_ms*/ = 2000) override {
    last_kill_target = executable_basename;
    int removed = 0;
    for (auto it = table_.begin(); it != table_.end();) {
      if (it->executable == executable_basename) {
        it = table_.erase(it);
        ++removed;
      } else {
        ++it;
      }
    }
    return removed;
  }

  bool kill_pid(int pid, const std::string & expected_basename, int /*timeout_ms*/ = 2000) override {
    for (auto it = table_.begin(); it != table_.end(); ++it) {
      if (it->pid == pid && it->executable == expected_basename) {
        table_.erase(it);
        return true;
      }
    }
    return false;
  }

  tl::expected<int, std::string> spawn(const std::string & executable_path) override {
    const auto slash = executable_path.rfind('/');
    const std::string base = (slash == std::string::npos) ? executable_path : executable_path.substr(slash + 1);
    const int pid = next_fake_pid_++;
    table_.push_back(Entry{pid, base});
    return pid;
  }

  int count(const std::string & executable_basename) const {
    int n = 0;
    for (const auto & e : table_) {
      if (e.executable == executable_basename) {
        ++n;
      }
    }
    return n;
  }

  std::vector<std::string> live_executables() const {
    std::vector<std::string> out;
    out.reserve(table_.size());
    for (const auto & e : table_) {
      out.push_back(e.executable);
    }
    return out;
  }

 private:
  std::vector<Entry> table_;
  int next_fake_pid_ = 1000;
};

ros2_medkit_gateway::UpdateProgressReporter make_reporter(ros2_medkit_gateway::UpdateStatusInfo & info,
                                                         std::mutex & mu) {
  return ros2_medkit_gateway::UpdateProgressReporter(info, mu);
}

// Build a valid .tar.gz containing <pkg>/lib/<pkg>/<exe> (a tiny stub) and
// return its raw bytes, or "" if the system tar is unavailable. The plugin
// under test extracts via fork+execvp; this fixture setup uses the shell tar.
std::string build_artifact_bytes(const std::string & pkg, const std::string & exe) {
  namespace fs = std::filesystem;
  std::error_code ec;
  const std::string work = ::testing::TempDir() + "/ota_fixture_" + pkg + "_" + exe;
  fs::remove_all(work, ec);
  fs::create_directories(work + "/" + pkg + "/lib/" + pkg, ec);
  {
    std::ofstream f(work + "/" + pkg + "/lib/" + pkg + "/" + exe, std::ios::binary);
    f << "#!/bin/sh\nexit 0\n";
  }
  const std::string tgz = work + ".tar.gz";
  const std::string cmd = "tar -czf '" + tgz + "' -C '" + work + "' '" + pkg + "'";
  if (std::system(cmd.c_str()) != 0) {
    return {};
  }
  std::ifstream in(tgz, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

}  // namespace

TEST(OtaUpdatePluginSmoke, NameAndConstructible) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  EXPECT_EQ(plugin.name(), "ota_update_plugin");
}

// Exercises the real ProcessRunner (not the FakeProcessRunner double)
// through the actual double-fork + status pipe in process_runner.cpp.
// Before the pipe2/errno fix, spawn() unconditionally returned the
// (already-reaped) intermediate child's pid regardless of whether execl
// succeeded, so this exact case - a path that cannot exist - reported
// success with a pid that was never running.
TEST(ProcessRunnerSpawn, NonexistentExecutableReturnsError) {
  ota_update_plugin::ProcessRunner runner;
  auto rc = runner.spawn("/nonexistent/path/does-not-exist-ota-demo");
  ASSERT_FALSE(rc);
  EXPECT_NE(rc.error().find("execl failed"), std::string::npos) << "unexpected error message: " << rc.error();
}

TEST(OtaUpdatePluginSmoke, RegisterListGet) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  nlohmann::json md = {{"id", "u1"}, {"updated_components", {"x"}}};
  ASSERT_TRUE(plugin.register_update(md));
  auto ids = plugin.list_updates({});
  ASSERT_TRUE(ids);
  ASSERT_EQ(ids->size(), 1u);
  EXPECT_EQ((*ids)[0], "u1");
  auto got = plugin.get_update("u1");
  ASSERT_TRUE(got);
  EXPECT_EQ((*got).content["id"], "u1");
}

TEST(OtaUpdatePluginSmoke, RegisterRequiresId) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  auto rc = plugin.register_update(nlohmann::json::object());
  EXPECT_FALSE(rc);
  EXPECT_EQ(rc.error().code, ros2_medkit_gateway::UpdateBackendError::InvalidInput);
}

TEST(OtaUpdatePluginSmoke, GetUpdateReturnsNotFoundForUnknownId) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  auto got = plugin.get_update("does-not-exist");
  ASSERT_FALSE(got);
  EXPECT_EQ(got.error().code, ros2_medkit_gateway::UpdateBackendError::NotFound);
}

TEST(OtaUpdatePluginSmoke, DeleteRemovesEntry) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  ASSERT_TRUE(plugin.register_update({{"id", "to-delete"}, {"updated_components", {"x"}}}));
  ASSERT_TRUE(plugin.delete_update("to-delete"));
  auto got = plugin.get_update("to-delete");
  EXPECT_FALSE(got);
}

TEST(OtaUpdatePluginSmoke, BootPollPopulates) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  plugin.configure(nlohmann::json::object());
  auto fake = std::make_unique<FakeCatalogClient>("http://x");
  fake->catalog_payload = nlohmann::json::array({
      {{"id", "a"},
       {"updated_components", {"scan"}},
       {"x_medkit_artifact_url", "/artifacts/a.tgz"},
       {"x_medkit_target_package", "a"}},
  });
  plugin.set_catalog_client_for_test(std::move(fake));
  plugin.poll_and_register_catalog();

  auto ids = plugin.list_updates({});
  ASSERT_TRUE(ids);
  ASSERT_EQ(ids->size(), 1u);
  EXPECT_EQ((*ids)[0], "a");
}

TEST(OtaUpdatePluginSmoke, PrepareRejectsUnknownOperationKind) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  ASSERT_TRUE(plugin.register_update({{"id", "bad"}}));
  ros2_medkit_gateway::UpdateStatusInfo info;
  std::mutex mu;
  auto reporter = make_reporter(info, mu);
  auto rc = plugin.prepare("bad", reporter);
  ASSERT_FALSE(rc);
  EXPECT_EQ(rc.error().code, ros2_medkit_gateway::UpdateBackendError::InvalidInput);
}

TEST(OtaUpdatePluginSmoke, PrepareUninstallSkipsDownload) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  plugin.configure(nlohmann::json::object());
  // No download should happen for uninstall, but provide a fake just in case.
  auto fake = std::make_unique<FakeCatalogClient>("http://x");
  plugin.set_catalog_client_for_test(std::move(fake));
  ASSERT_TRUE(plugin.register_update({{"id", "rm"}, {"removed_components", {"legacy"}}}));

  ros2_medkit_gateway::UpdateStatusInfo info;
  std::mutex mu;
  auto reporter = make_reporter(info, mu);
  auto rc = plugin.prepare("rm", reporter);
  EXPECT_TRUE(rc);
  EXPECT_EQ(info.progress.value_or(-1), 100);
}

TEST(OtaUpdatePluginSmoke, ExecuteUpdateUsesReplacesExecutableForKill) {
  const std::string bytes = build_artifact_bytes("fixed_lidar", "fixed_lidar_node");
  if (bytes.empty()) GTEST_SKIP() << "system tar unavailable to build fixture tarball";

  ota_update_plugin::OtaUpdatePlugin plugin;
  plugin.configure({{"staging_dir", ::testing::TempDir() + "/replaces_test/staging"},
                    {"install_dir", ::testing::TempDir() + "/replaces_test/install"}});
  auto fake = std::make_unique<FakeCatalogClient>("http://x");
  fake->artifact_to_return = bytes;
  plugin.set_catalog_client_for_test(std::move(fake));
  auto runner = std::make_unique<FakeProcessRunner>();
  FakeProcessRunner * runner_raw = runner.get();
  plugin.set_process_runner_for_test(std::move(runner));

  // Update entry with separate old + new executable basenames.
  ASSERT_TRUE(plugin.register_update({
      {"id", "u_replaces"},
      {"updated_components", {"scan_sensor_node"}},
      {"x_medkit_artifact_url", "/artifacts/fixed.tgz"},
      {"x_medkit_target_package", "fixed_lidar"},
      {"x_medkit_executable", "fixed_lidar_node"},
      {"x_medkit_replaces_executable", "broken_lidar_node"},
  }));

  ros2_medkit_gateway::UpdateStatusInfo info;
  std::mutex mu;
  auto reporter = make_reporter(info, mu);
  ASSERT_TRUE(plugin.prepare("u_replaces", reporter));

  // With stage-then-swap the extract now succeeds first (real tarball), so the
  // additive kill runs and still targets x_medkit_replaces_executable.
  auto rc = plugin.execute("u_replaces", reporter);
  ASSERT_TRUE(rc) << rc.error().message;
  EXPECT_EQ(runner_raw->last_kill_target, "broken_lidar_node");
  EXPECT_EQ(runner_raw->count("fixed_lidar_node"), 1);
}

TEST(OtaUpdatePluginSmoke, ExecuteUpdateFallsBackToExecutableWhenReplacesMissing) {
  const std::string bytes = build_artifact_bytes("scan_pkg", "scan_node");
  if (bytes.empty()) GTEST_SKIP() << "system tar unavailable to build fixture tarball";

  ota_update_plugin::OtaUpdatePlugin plugin;
  plugin.configure({{"staging_dir", ::testing::TempDir() + "/replaces_fallback/staging"},
                    {"install_dir", ::testing::TempDir() + "/replaces_fallback/install"}});
  auto fake = std::make_unique<FakeCatalogClient>("http://x");
  fake->artifact_to_return = bytes;
  plugin.set_catalog_client_for_test(std::move(fake));
  auto runner = std::make_unique<FakeProcessRunner>();
  FakeProcessRunner * runner_raw = runner.get();
  plugin.set_process_runner_for_test(std::move(runner));

  // Update entry without x_medkit_replaces_executable - kill should target
  // the same name as x_medkit_executable.
  ASSERT_TRUE(plugin.register_update({
      {"id", "u_no_replaces"},
      {"updated_components", {"scan_sensor_node"}},
      {"x_medkit_artifact_url", "/artifacts/scan.tgz"},
      {"x_medkit_target_package", "scan_pkg"},
      {"x_medkit_executable", "scan_node"},
  }));

  ros2_medkit_gateway::UpdateStatusInfo info;
  std::mutex mu;
  auto reporter = make_reporter(info, mu);
  ASSERT_TRUE(plugin.prepare("u_no_replaces", reporter));

  auto rc = plugin.execute("u_no_replaces", reporter);
  ASSERT_TRUE(rc) << rc.error().message;
  EXPECT_EQ(runner_raw->last_kill_target, "scan_node");
}

// --- Path-traversal rejection (findings #2/#8/#13) ---

TEST(OtaUpdatePluginSmoke, ExecuteRejectsTargetPackageTraversal) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  plugin.configure({{"staging_dir", ::testing::TempDir() + "/trav_pkg/staging"},
                    {"install_dir", ::testing::TempDir() + "/trav_pkg/install"}});
  plugin.set_catalog_client_for_test(std::make_unique<FakeCatalogClient>("http://x"));
  auto runner = std::make_unique<FakeProcessRunner>();
  FakeProcessRunner * runner_raw = runner.get();
  plugin.set_process_runner_for_test(std::move(runner));

  ASSERT_TRUE(plugin.register_update({
      {"id", "u_trav_pkg"},
      {"updated_components", {"scan_sensor_node"}},
      {"x_medkit_artifact_url", "/artifacts/x.tgz"},
      {"x_medkit_target_package", "../../etc/x"},
      {"x_medkit_executable", "scan_node"},
  }));

  ros2_medkit_gateway::UpdateStatusInfo info;
  std::mutex mu;
  auto reporter = make_reporter(info, mu);
  ASSERT_TRUE(plugin.prepare("u_trav_pkg", reporter));
  auto rc = plugin.execute("u_trav_pkg", reporter);
  ASSERT_FALSE(rc);
  EXPECT_EQ(rc.error().code, ros2_medkit_gateway::UpdateBackendError::InvalidInput);
  // Rejected before any process action or filesystem swap - no kill, no spawn.
  EXPECT_TRUE(runner_raw->last_kill_target.empty());
  EXPECT_TRUE(runner_raw->live_executables().empty());
}

TEST(OtaUpdatePluginSmoke, ExecuteRejectsExecutableTraversal) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  plugin.configure({{"staging_dir", ::testing::TempDir() + "/trav_exe/staging"},
                    {"install_dir", ::testing::TempDir() + "/trav_exe/install"}});
  plugin.set_catalog_client_for_test(std::make_unique<FakeCatalogClient>("http://x"));
  auto runner = std::make_unique<FakeProcessRunner>();
  FakeProcessRunner * runner_raw = runner.get();
  plugin.set_process_runner_for_test(std::move(runner));

  ASSERT_TRUE(plugin.register_update({
      {"id", "u_trav_exe"},
      {"updated_components", {"scan_sensor_node"}},
      {"x_medkit_artifact_url", "/artifacts/x.tgz"},
      {"x_medkit_target_package", "fixed_lidar"},
      {"x_medkit_executable", "../../bin/sh"},
  }));

  ros2_medkit_gateway::UpdateStatusInfo info;
  std::mutex mu;
  auto reporter = make_reporter(info, mu);
  ASSERT_TRUE(plugin.prepare("u_trav_exe", reporter));
  auto rc = plugin.execute("u_trav_exe", reporter);
  ASSERT_FALSE(rc);
  EXPECT_EQ(rc.error().code, ros2_medkit_gateway::UpdateBackendError::InvalidInput);
  EXPECT_TRUE(runner_raw->last_kill_target.empty());
  EXPECT_TRUE(runner_raw->live_executables().empty());
}

// --- Stage-then-swap: don't kill before a verified extract (finding #7) ---

TEST(OtaUpdatePluginSmoke, ExtractFailureDoesNotKill) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  plugin.configure({{"staging_dir", ::testing::TempDir() + "/extfail/staging"},
                    {"install_dir", ::testing::TempDir() + "/extfail/install"}});
  auto fake = std::make_unique<FakeCatalogClient>("http://x");
  fake->artifact_to_return = "this is not a gzip archive";  // extraction will fail
  plugin.set_catalog_client_for_test(std::move(fake));
  auto runner = std::make_unique<FakeProcessRunner>();
  FakeProcessRunner * runner_raw = runner.get();
  // Pre-existing lidar node (as if spawned by the launch file, not the plugin).
  ASSERT_TRUE(runner_raw->spawn("/opt/ros/lib/broken_lidar/broken_lidar_node"));
  plugin.set_process_runner_for_test(std::move(runner));

  ASSERT_TRUE(plugin.register_update({
      {"id", "u_extfail"},
      {"updated_components", {"scan_sensor_node"}},
      {"x_medkit_artifact_url", "/artifacts/broken.tgz"},
      {"x_medkit_target_package", "fixed_lidar"},
      {"x_medkit_executable", "fixed_lidar_node"},
      {"x_medkit_replaces_executable", "broken_lidar_node"},
  }));

  ros2_medkit_gateway::UpdateStatusInfo info;
  std::mutex mu;
  auto reporter = make_reporter(info, mu);
  ASSERT_TRUE(plugin.prepare("u_extfail", reporter));
  auto rc = plugin.execute("u_extfail", reporter);
  ASSERT_FALSE(rc);  // extract failed
  // The running lidar was NOT killed - extract happens before any kill.
  EXPECT_TRUE(runner_raw->last_kill_target.empty());
  EXPECT_EQ(runner_raw->count("broken_lidar_node"), 1);
}

// --- Track what we spawned; additive re-apply dedup (findings #9/#10) ---

TEST(OtaUpdatePluginSmoke, CrossPackageReexecuteDoesNotDuplicate) {
  const std::string bytes = build_artifact_bytes("fixed_lidar", "fixed_lidar_node");
  if (bytes.empty()) GTEST_SKIP() << "system tar unavailable to build fixture tarball";

  ota_update_plugin::OtaUpdatePlugin plugin;
  plugin.configure({{"staging_dir", ::testing::TempDir() + "/reexec/staging"},
                    {"install_dir", ::testing::TempDir() + "/reexec/install"}});
  auto fake = std::make_unique<FakeCatalogClient>("http://x");
  fake->artifact_to_return = bytes;
  plugin.set_catalog_client_for_test(std::move(fake));
  auto runner = std::make_unique<FakeProcessRunner>();
  FakeProcessRunner * runner_raw = runner.get();
  plugin.set_process_runner_for_test(std::move(runner));

  ASSERT_TRUE(plugin.register_update({
      {"id", "u_reexec"},
      {"updated_components", {"scan_sensor_node"}},
      {"x_medkit_artifact_url", "/artifacts/fixed.tgz"},
      {"x_medkit_target_package", "fixed_lidar"},
      {"x_medkit_executable", "fixed_lidar_node"},
      {"x_medkit_replaces_executable", "broken_lidar_node"},
  }));

  ros2_medkit_gateway::UpdateStatusInfo info;
  std::mutex mu;
  auto reporter = make_reporter(info, mu);
  ASSERT_TRUE(plugin.prepare("u_reexec", reporter));
  ASSERT_TRUE(plugin.execute("u_reexec", reporter));
  EXPECT_EQ(runner_raw->count("fixed_lidar_node"), 1);
  // Re-apply the same update: the previously plugin-spawned node must be killed
  // by recorded pid so we do not accumulate duplicates.
  ASSERT_TRUE(plugin.execute("u_reexec", reporter));
  EXPECT_EQ(runner_raw->count("fixed_lidar_node"), 1);
}

// --- Uninstall kill + fragment (findings #11/#12) ---

TEST(OtaUpdatePluginSmoke, UninstallKillsRecordedExecutableNotPackage) {
  const std::string bytes = build_artifact_bytes("newpkg", "new_node");
  if (bytes.empty()) GTEST_SKIP() << "system tar unavailable to build fixture tarball";

  ota_update_plugin::OtaUpdatePlugin plugin;
  plugin.configure({{"staging_dir", ::testing::TempDir() + "/uninstall_kill/staging"},
                    {"install_dir", ::testing::TempDir() + "/uninstall_kill/install"}});
  auto fake = std::make_unique<FakeCatalogClient>("http://x");
  fake->artifact_to_return = bytes;
  plugin.set_catalog_client_for_test(std::move(fake));
  auto runner = std::make_unique<FakeProcessRunner>();
  FakeProcessRunner * runner_raw = runner.get();
  plugin.set_process_runner_for_test(std::move(runner));

  ros2_medkit_gateway::UpdateStatusInfo info;
  std::mutex mu;
  auto reporter = make_reporter(info, mu);

  // Install records the spawned executable under component "newapp".
  ASSERT_TRUE(plugin.register_update({
      {"id", "i_new"},
      {"added_components", {"newapp"}},
      {"x_medkit_artifact_url", "/artifacts/new.tgz"},
      {"x_medkit_target_package", "newpkg"},
      {"x_medkit_executable", "new_node"},
  }));
  ASSERT_TRUE(plugin.prepare("i_new", reporter));
  ASSERT_TRUE(plugin.execute("i_new", reporter));
  ASSERT_EQ(runner_raw->count("new_node"), 1);

  // Uninstall the same component: the recorded executable is killed, NOT the
  // package basename.
  ASSERT_TRUE(plugin.register_update({
      {"id", "rm_new"},
      {"removed_components", {"newapp"}},
      {"x_medkit_target_package", "newpkg"},
  }));
  ASSERT_TRUE(plugin.prepare("rm_new", reporter));
  ASSERT_TRUE(plugin.execute("rm_new", reporter));
  EXPECT_EQ(runner_raw->count("new_node"), 0);
  // Uninstall targets the recorded pid via kill_pid, never kill_by_executable
  // on the package name.
  EXPECT_NE(runner_raw->last_kill_target, "newpkg");
}

TEST(OtaUpdatePluginSmoke, UninstallRemovesFragmentByComponent) {
  const std::string bytes = build_artifact_bytes("fragpkg", "frag_node");
  if (bytes.empty()) GTEST_SKIP() << "system tar unavailable to build fixture tarball";

  namespace fs = std::filesystem;
  const std::string frag_dir = ::testing::TempDir() + "/frag_component/fragments";
  std::error_code ec;
  fs::remove_all(frag_dir, ec);

  ota_update_plugin::OtaUpdatePlugin plugin;
  plugin.configure({{"staging_dir", ::testing::TempDir() + "/frag_component/staging"},
                    {"install_dir", ::testing::TempDir() + "/frag_component/install"},
                    {"fragments_dir", frag_dir}});
  auto fake = std::make_unique<FakeCatalogClient>("http://x");
  fake->artifact_to_return = bytes;
  plugin.set_catalog_client_for_test(std::move(fake));
  plugin.set_process_runner_for_test(std::make_unique<FakeProcessRunner>());

  ros2_medkit_gateway::UpdateStatusInfo info;
  std::mutex mu;
  auto reporter = make_reporter(info, mu);

  // Install writes <component>.yaml keyed on added_components[0].
  ASSERT_TRUE(plugin.register_update({
      {"id", "i_frag"},
      {"added_components", {"frag_app"}},
      {"x_medkit_artifact_url", "/artifacts/frag.tgz"},
      {"x_medkit_target_package", "fragpkg"},
      {"x_medkit_executable", "frag_node"},
  }));
  ASSERT_TRUE(plugin.prepare("i_frag", reporter));
  ASSERT_TRUE(plugin.execute("i_frag", reporter));
  EXPECT_TRUE(fs::exists(frag_dir + "/frag_app.yaml"));

  // Uninstall removes the SAME file, keyed on removed_components[0].
  ASSERT_TRUE(plugin.register_update({
      {"id", "rm_frag"},
      {"removed_components", {"frag_app"}},
      {"x_medkit_target_package", "fragpkg"},
  }));
  ASSERT_TRUE(plugin.prepare("rm_frag", reporter));
  ASSERT_TRUE(plugin.execute("rm_frag", reporter));
  EXPECT_FALSE(fs::exists(frag_dir + "/frag_app.yaml"));

  int remaining = 0;
  for (const auto & entry : fs::directory_iterator(frag_dir, ec)) {
    (void)entry;
    ++remaining;
  }
  EXPECT_EQ(remaining, 0);
}

// --- Fragment YAML quoting (finding #13-yaml) ---

TEST(OtaFragmentRender, RenderFragmentQuotesScalars) {
  const std::string body = ota_update_plugin::detail::render_install_fragment(
      "scan node: 1", "lidar node", "OTA-installed via u\ninjected_top: true");
  // Values with spaces/colons are emitted as double-quoted scalars.
  EXPECT_NE(body.find("\"scan node: 1\""), std::string::npos);
  EXPECT_NE(body.find("\"lidar node\""), std::string::npos);
  // A newline in a value is escaped, so it cannot open a new top-level YAML key.
  EXPECT_EQ(body.find("\ninjected_top:"), std::string::npos);
  EXPECT_NE(body.find("\\ninjected_top: true"), std::string::npos);
}
