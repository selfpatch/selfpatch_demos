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

#include <string>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace ota_update_plugin {

/// Decomposed URL components used by the HTTP client.
struct ParsedUrl {
  std::string host;
  int port;
  bool tls;
  std::string path;
};

/// Parse an http:// or https:// URL into components.
/// Throws std::invalid_argument for unsupported schemes.
ParsedUrl parse_url(const std::string & url);

/// HTTP client that fetches the FastAPI catalog and downloads artifacts.
/// Virtual methods so tests can substitute a fake without touching real HTTP.
class CatalogClient {
 public:
  explicit CatalogClient(std::string base_url);
  virtual ~CatalogClient() = default;

  CatalogClient(const CatalogClient &) = delete;
  CatalogClient & operator=(const CatalogClient &) = delete;
  CatalogClient(CatalogClient &&) = delete;
  CatalogClient & operator=(CatalogClient &&) = delete;

  /// GET {base_url}/catalog and parse JSON. Returns the JSON array on success.
  virtual tl::expected<nlohmann::json, std::string> fetch_catalog();

  /// Download an artifact. `url_or_path` may be either an absolute URL or a
  /// path (interpreted relative to `base_url`). Body is written to `out_path`.
  /// Returns the absolute output path on success.
  virtual tl::expected<std::string, std::string> download_artifact(const std::string & url_or_path,
                                                                   const std::string & out_path);

 protected:
  std::string base_url_;
};

}  // namespace ota_update_plugin
