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
#include <vector>

#include <tl/expected.hpp>

namespace ota_update_plugin {

/// Process management helper for OTA operations: locate, terminate, and spawn
/// demo nodes by executable basename. Pure-virtual for test substitution.
class ProcessRunner {
 public:
  ProcessRunner() = default;
  virtual ~ProcessRunner() = default;

  ProcessRunner(const ProcessRunner &) = delete;
  ProcessRunner & operator=(const ProcessRunner &) = delete;
  ProcessRunner(ProcessRunner &&) = delete;
  ProcessRunner & operator=(ProcessRunner &&) = delete;

  /// Find PIDs of processes whose /proc/<pid>/comm matches the given basename.
  virtual std::vector<int> pgrep(const std::string & executable_basename);

  /// Send SIGTERM to all matching PIDs, wait up to `timeout_ms` for exit, then
  /// SIGKILL any stragglers. Returns the number of processes that were signalled.
  virtual tl::expected<int, std::string> kill_by_executable(const std::string & executable_basename,
                                                            int timeout_ms = 2000);

  /// fork+exec the executable at `executable_path`. Returns child PID or error.
  virtual tl::expected<int, std::string> spawn(const std::string & executable_path);
};

}  // namespace ota_update_plugin
