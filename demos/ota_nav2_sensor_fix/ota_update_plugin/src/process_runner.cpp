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
#include <fcntl.h>
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

// /proc/<pid>/comm is truncated to 15 characters by the kernel, which causes
// false negatives for any executable whose basename is longer (e.g.
// "broken_lidar_node" -> "broken_lidar_no"). Read /proc/<pid>/cmdline
// instead - its first NUL-separated arg holds the full path / argv[0].
std::string proc_cmdline_arg0(int pid) {
  std::ifstream f("/proc/" + std::to_string(pid) + "/cmdline", std::ios::binary);
  if (!f) {
    return {};
  }
  std::string buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (buf.empty()) {
    return {};
  }
  // argv[0] runs to the first NUL.
  const auto nul = buf.find('\0');
  std::string arg0 = (nul == std::string::npos) ? buf : buf.substr(0, nul);
  // Take the basename so callers pass executable_basename without a path.
  const auto slash = arg0.rfind('/');
  return (slash == std::string::npos) ? arg0 : arg0.substr(slash + 1);
}

bool is_pid_dir(const char * name) {
  for (const char * p = name; *p; ++p) {
    if (*p < '0' || *p > '9') {
      return false;
    }
  }
  return *name != '\0';
}

// Read exactly `len` bytes from `fd`, retrying on EINTR and short reads.
// Returns the number of bytes actually read: `len` on a full read, a smaller
// count (including 0) if EOF is hit first, or -1 on a read() error other than
// EINTR. Used by the parent side of spawn()'s two status pipes, where a
// short/zero read from `pid_fds` (the *first* pipe) is itself an error
// condition (the intermediate child exited without reporting), while a
// short/zero read from `err_fds` (the *second* pipe) is the success signal
// (EOF via O_CLOEXEC).
ssize_t read_exact(int fd, void * buf, size_t len) {
  size_t total = 0;
  auto * p = static_cast<unsigned char *>(buf);
  while (total < len) {
    const ssize_t n = ::read(fd, p + total, len - total);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    }
    if (n == 0) {
      break;  // EOF
    }
    total += static_cast<size_t>(n);
  }
  return static_cast<ssize_t>(total);
}

// Best-effort write of `len` bytes to `fd`, retrying on EINTR and short
// writes. Called from a child right before `_exit()`, so there is nothing
// actionable to do on failure - the parent's read side already treats a
// missing/short read as an error, so a failed write here cannot manifest as
// a silent false success.
void write_best_effort(int fd, const void * buf, size_t len) {
  const auto * p = static_cast<const unsigned char *>(buf);
  size_t total = 0;
  while (total < len) {
    const ssize_t n = ::write(fd, p + total, len - total);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return;
    }
    if (n == 0) {
      return;
    }
    total += static_cast<size_t>(n);
  }
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
    if (proc_cmdline_arg0(pid) == executable_basename) {
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

bool ProcessRunner::kill_pid(int pid, const std::string & expected_basename, int timeout_ms) {
  if (pid <= 0) {
    return false;
  }
  // Existence probe. A dead pid (or one we do not own) fails here.
  if (::kill(pid, 0) != 0) {
    return false;
  }
  // Pid-reuse guard: refuse to signal a pid the kernel has recycled onto an
  // unrelated process. argv[0] basename must still match what we spawned.
  if (proc_cmdline_arg0(pid) != expected_basename) {
    return false;
  }
  if (::kill(pid, SIGTERM) != 0) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (::kill(pid, 0) != 0) {
      return true;  // exited on its own
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (::kill(pid, 0) == 0) {
    ::kill(pid, SIGKILL);
  }
  return true;
}

tl::expected<int, std::string> ProcessRunner::spawn(const std::string & executable_path) {
  // Double-fork so the grandchild is reparented to init and never becomes a
  // zombie in the gateway process. The intermediate child exits immediately
  // and is reaped here.
  //
  // Two separate status pipes carry the outcome back to the parent, each
  // with exactly one writer, so fixed-offset reads can never be misordered:
  //   - pid_fds: the intermediate child writes the grandchild's pid, then
  //     exits. The grandchild never writes to this pipe.
  //   - err_fds: the grandchild writes its errno here ONLY if execl fails,
  //     then exits 127. On successful exec, its O_CLOEXEC copy of the write
  //     end closes automatically, so the parent's read sees EOF. The
  //     intermediate child never writes to this pipe.
  // (A single shared pipe for both messages was tried before and reverted:
  // both writes are the same size (4 bytes), so nothing guarantees the pid
  // arrives before the errno on a very fast exec failure - the parent's two
  // fixed-offset reads could then swap them, turning a real errno into a
  // bogus strerror() of a pid.)
  int pid_fds[2];
  if (::pipe2(pid_fds, O_CLOEXEC) != 0) {
    return tl::make_unexpected(std::string("pipe2 failed: ") + std::strerror(errno));
  }
  int err_fds[2];
  if (::pipe2(err_fds, O_CLOEXEC) != 0) {
    const int err = errno;
    ::close(pid_fds[0]);
    ::close(pid_fds[1]);
    return tl::make_unexpected(std::string("pipe2 failed: ") + std::strerror(err));
  }

  pid_t pid = fork();
  if (pid < 0) {
    const int err = errno;
    ::close(pid_fds[0]);
    ::close(pid_fds[1]);
    ::close(err_fds[0]);
    ::close(err_fds[1]);
    return tl::make_unexpected(std::string("fork failed: ") + std::strerror(err));
  }
  if (pid == 0) {
    // Intermediate child: only ever writes to pid_fds. Close the read ends
    // of both pipes now - it needs neither. err_fds[1] must stay open
    // across the second fork() so the grandchild inherits it (that's the
    // only way the grandchild gets a copy to write its errno to); the
    // intermediate itself never writes to err_fds and its own copy closes
    // automatically when it `_exit`s below, without a race (the parent
    // `waitpid`s the intermediate before reading either pipe).
    ::close(pid_fds[0]);
    ::close(err_fds[0]);
    pid_t grandchild = fork();
    if (grandchild < 0) {
      _exit(126);
    }
    if (grandchild == 0) {
      // Grandchild: only ever writes to err_fds (and only on exec failure),
      // so drop pid_fds's write end - it must never write the pid pipe.
      ::close(pid_fds[1]);
      setsid();
      // Forward use_sim_time so the spawned node aligns with the
      // gateway's clock domain. Without this, a node started by OTA
      // (post-update fixed_lidar) runs on wall time while the rest of
      // the stack runs on /clock from gz-sim - its /scan / /diagnostics
      // timestamps fall outside nav2's TF buffer and the costmap drops
      // every message: "the timestamp on the message is earlier than
      // all the data in the transform cache". Robot stops responding.
      //
      // Note: this is the minimum viable param plumbing. A full
      // production plugin should plumb arbitrary parameters from
      // the catalog entry through to execve.
      execl(executable_path.c_str(),
            executable_path.c_str(),
            "--ros-args",
            "-p", "use_sim_time:=true",
            static_cast<char *>(nullptr));
      const int err = errno;
      std::fprintf(stderr, "execl %s failed: %s\n", executable_path.c_str(), std::strerror(err));
      write_best_effort(err_fds[1], &err, sizeof(err));
      _exit(127);
    }
    write_best_effort(pid_fds[1], &grandchild, sizeof(grandchild));
    _exit(0);
  }

  // Parent: drop both write ends immediately - if either stayed open here,
  // its pipe could never signal EOF (the read below would block forever
  // even after both children released their copies).
  ::close(pid_fds[1]);
  ::close(err_fds[1]);
  int status = 0;
  ::waitpid(pid, &status, 0);

  pid_t grandchild_pid = -1;
  const ssize_t pid_bytes = read_exact(pid_fds[0], &grandchild_pid, sizeof(grandchild_pid));
  ::close(pid_fds[0]);
  if (pid_bytes != static_cast<ssize_t>(sizeof(grandchild_pid))) {
    ::close(err_fds[0]);
    return tl::make_unexpected(
        std::string("spawn failed: intermediate child exited without reporting a pid "
                     "(second fork likely failed)"));
  }

  int exec_errno = 0;
  const ssize_t err_bytes = read_exact(err_fds[0], &exec_errno, sizeof(exec_errno));
  ::close(err_fds[0]);
  if (err_bytes == 0) {
    // EOF: the grandchild's copy of the write end closed (via O_CLOEXEC)
    // without reporting an errno - exec succeeded.
    return static_cast<int>(grandchild_pid);
  }
  if (err_bytes == static_cast<ssize_t>(sizeof(exec_errno))) {
    return tl::make_unexpected(std::string("execl failed: ") + std::strerror(exec_errno));
  }
  return tl::make_unexpected(std::string("spawn failed: unexpected status pipe read"));
}

}  // namespace ota_update_plugin
