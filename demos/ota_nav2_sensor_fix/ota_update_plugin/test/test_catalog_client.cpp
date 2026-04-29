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

#include <stdexcept>

#include <gtest/gtest.h>

#include "catalog_client.hpp"

using ota_update_plugin::parse_url;

TEST(ParseUrl, HostAndPort) {
  auto p = parse_url("http://server:9000");
  EXPECT_EQ(p.host, "server");
  EXPECT_EQ(p.port, 9000);
  EXPECT_FALSE(p.tls);
  EXPECT_EQ(p.path, "/");
}

TEST(ParseUrl, PathSplit) {
  auto p = parse_url("http://server:9000/catalog");
  EXPECT_EQ(p.host, "server");
  EXPECT_EQ(p.port, 9000);
  EXPECT_EQ(p.path, "/catalog");
}

TEST(ParseUrl, DefaultsHttpPort) {
  auto p = parse_url("http://server/catalog");
  EXPECT_EQ(p.host, "server");
  EXPECT_EQ(p.port, 80);
  EXPECT_FALSE(p.tls);
  EXPECT_EQ(p.path, "/catalog");
}

TEST(ParseUrl, HttpsTls) {
  auto p = parse_url("https://server/catalog");
  EXPECT_TRUE(p.tls);
  EXPECT_EQ(p.port, 443);
  EXPECT_EQ(p.path, "/catalog");
}

TEST(ParseUrl, RejectsInvalidScheme) {
  EXPECT_THROW(parse_url("ftp://server/foo"), std::invalid_argument);
}

TEST(ParseUrl, RejectsMissingHost) {
  EXPECT_THROW(parse_url("http://:9000/foo"), std::invalid_argument);
}
