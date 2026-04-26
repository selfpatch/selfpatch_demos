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

#include "process_runner.hpp"

#include <dirent.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <thread>

namespace ota_update_plugin {

namespace {

std::string proc_comm(int pid) {
  std::ifstream f("/proc/" + std::to_string(pid) + "/comm");
  if (!f) {
    return {};
  }
  std::string line;
  std::getline(f, line);
  return line;
}

bool is_pid_dir(const char * name) {
  for (const char * p = name; *p; ++p) {
    if (*p < '0' || *p > '9') {
      return false;
    }
  }
  return *name != '\0';
}

}  // namespace

std::vector<int> ProcessRunner::pgrep(const std::string & executable_basename) {
  std::vector<int> out;
  DIR * d = opendir("/proc");
  if (d == nullptr) {
    return out;
  }
  while (auto * ent = readdir(d)) {
    if (!is_pid_dir(ent->d_name)) {
      continue;
    }
    const int pid = std::atoi(ent->d_name);
    if (pid <= 0) {
      continue;
    }
    if (proc_comm(pid) == executable_basename) {
      out.push_back(pid);
    }
  }
  closedir(d);
  return out;
}

tl::expected<int, std::string> ProcessRunner::kill_by_executable(const std::string & executable_basename,
                                                                 int timeout_ms) {
  const auto pids = pgrep(executable_basename);
  int signalled = 0;
  for (int pid : pids) {
    if (::kill(pid, SIGTERM) == 0) {
      ++signalled;
    }
  }
  if (signalled == 0) {
    return 0;
  }

  // Poll for exit.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    bool any_alive = false;
    for (int pid : pids) {
      if (::kill(pid, 0) == 0) {
        any_alive = true;
        break;
      }
    }
    if (!any_alive) {
      return signalled;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  // Force-kill stragglers.
  for (int pid : pids) {
    if (::kill(pid, 0) == 0) {
      ::kill(pid, SIGKILL);
    }
  }
  return signalled;
}

tl::expected<int, std::string> ProcessRunner::spawn(const std::string & executable_path) {
  // Double-fork so the grandchild is reparented to init and never becomes a
  // zombie in the gateway process. The intermediate child exits immediately
  // and is reaped here.
  pid_t pid = fork();
  if (pid < 0) {
    return tl::make_unexpected(std::string("fork failed: ") + std::strerror(errno));
  }
  if (pid == 0) {
    pid_t grandchild = fork();
    if (grandchild < 0) {
      _exit(126);
    }
    if (grandchild == 0) {
      setsid();
      execl(executable_path.c_str(), executable_path.c_str(), nullptr);
      std::fprintf(stderr, "execl %s failed: %s\n", executable_path.c_str(),
                   std::strerror(errno));
      _exit(127);
    }
    _exit(0);
  }
  int status = 0;
  ::waitpid(pid, &status, 0);
  return static_cast<int>(pid);
}

}  // namespace ota_update_plugin
