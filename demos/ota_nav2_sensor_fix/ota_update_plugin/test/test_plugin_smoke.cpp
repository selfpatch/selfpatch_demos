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

#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include <ros2_medkit_gateway/updates/update_types.hpp>

#include "catalog_client.hpp"
#include "ota_update_plugin/ota_update_plugin.hpp"

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
    o << artifact_to_return;
    return out;
  }
};

ros2_medkit_gateway::UpdateProgressReporter make_reporter(ros2_medkit_gateway::UpdateStatusInfo & info,
                                                         std::mutex & mu) {
  return ros2_medkit_gateway::UpdateProgressReporter(info, mu);
}

}  // namespace

TEST(OtaUpdatePluginSmoke, NameAndConstructible) {
  ota_update_plugin::OtaUpdatePlugin plugin;
  EXPECT_EQ(plugin.name(), "ota_update_plugin");
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
  EXPECT_EQ((*got)["id"], "u1");
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
