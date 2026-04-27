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

#include "catalog_client.hpp"

#include <fstream>
#include <stdexcept>
#include <utility>

#include <httplib.h>

namespace ota_update_plugin {

namespace {

bool starts_with(const std::string & s, const std::string & prefix) {
  return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

}  // namespace

ParsedUrl parse_url(const std::string & url) {
  ParsedUrl out{};
  std::string rest;
  if (starts_with(url, "https://")) {
    out.tls = true;
    out.port = 443;
    rest = url.substr(8);
  } else if (starts_with(url, "http://")) {
    out.tls = false;
    out.port = 80;
    rest = url.substr(7);
  } else {
    throw std::invalid_argument("unsupported URL scheme: " + url);
  }

  // Split host[:port] from path.
  const auto slash = rest.find('/');
  std::string authority;
  if (slash == std::string::npos) {
    authority = rest;
    out.path = "/";
  } else {
    authority = rest.substr(0, slash);
    out.path = rest.substr(slash);
  }

  // Split host from port if present.
  const auto colon = authority.find(':');
  if (colon == std::string::npos) {
    out.host = authority;
  } else {
    out.host = authority.substr(0, colon);
    try {
      out.port = std::stoi(authority.substr(colon + 1));
    } catch (const std::exception & e) {
      throw std::invalid_argument(std::string("invalid port in URL: ") + url + " (" + e.what() + ")");
    }
  }

  if (out.host.empty()) {
    throw std::invalid_argument("missing host in URL: " + url);
  }
  return out;
}

CatalogClient::CatalogClient(std::string base_url) : base_url_(std::move(base_url)) {
}

tl::expected<nlohmann::json, std::string> CatalogClient::fetch_catalog() {
  ParsedUrl parsed;
  try {
    parsed = parse_url(base_url_);
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("invalid catalog url: ") + e.what());
  }

  if (parsed.tls) {
    return tl::make_unexpected("https not supported by demo CatalogClient");
  }

  // Strip trailing slash from base path, then append /catalog.
  std::string base_path = parsed.path;
  if (!base_path.empty() && base_path.back() == '/') {
    base_path.pop_back();
  }
  const std::string target = base_path + "/catalog";

  httplib::Client cli(parsed.host, parsed.port);
  cli.set_connection_timeout(5, 0);
  cli.set_read_timeout(5, 0);

  auto res = cli.Get(target.c_str());
  if (!res) {
    return tl::make_unexpected("catalog GET failed: " + httplib::to_string(res.error()));
  }
  if (res->status < 200 || res->status >= 300) {
    return tl::make_unexpected("catalog GET returned status " + std::to_string(res->status));
  }
  try {
    return nlohmann::json::parse(res->body);
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("catalog json parse failed: ") + e.what());
  }
}

tl::expected<std::string, std::string> CatalogClient::download_artifact(const std::string & url_or_path,
                                                                        const std::string & out_path) {
  // If url_or_path is an absolute URL, parse it directly. Otherwise treat as a
  // path relative to base_url_.
  std::string full_url;
  if (starts_with(url_or_path, "http://") || starts_with(url_or_path, "https://")) {
    full_url = url_or_path;
  } else {
    std::string base = base_url_;
    // Strip trailing slash on base, leading slash on relative path.
    while (!base.empty() && base.back() == '/') {
      base.pop_back();
    }
    std::string rel = url_or_path;
    if (rel.empty() || rel.front() != '/') {
      rel = "/" + rel;
    }
    full_url = base + rel;
  }

  ParsedUrl parsed;
  try {
    parsed = parse_url(full_url);
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("invalid artifact url: ") + e.what());
  }
  if (parsed.tls) {
    return tl::make_unexpected("https not supported by demo CatalogClient");
  }

  httplib::Client cli(parsed.host, parsed.port);
  cli.set_connection_timeout(5, 0);
  cli.set_read_timeout(30, 0);

  auto res = cli.Get(parsed.path.c_str());
  if (!res) {
    return tl::make_unexpected("artifact GET failed: " + httplib::to_string(res.error()));
  }
  if (res->status < 200 || res->status >= 300) {
    return tl::make_unexpected("artifact GET returned status " + std::to_string(res->status));
  }

  std::ofstream o(out_path, std::ios::binary);
  if (!o) {
    return tl::make_unexpected("cannot open output file: " + out_path);
  }
  o.write(res->body.data(), static_cast<std::streamsize>(res->body.size()));
  if (!o) {
    return tl::make_unexpected("write to output file failed: " + out_path);
  }
  return out_path;
}

}  // namespace ota_update_plugin
