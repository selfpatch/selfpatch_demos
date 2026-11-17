# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Stand-in for the gateway API and the fault manager's report service.

Runs inside the benchmark container where no Medkit runs (jitter control row). It
answers every GET with an empty collection at once, holds /faults/stream open like an
event stream, and accepts every ReportFault request at once, so the load generators
issue the same requests as against Medkit, with no Medkit behind them.
"""
from __future__ import annotations
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.srv import ReportFault


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_GET(self):  # noqa: N802
        if self.path.endswith("/faults/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.end_headers()
            try:
                while True:
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    time.sleep(1.0)
            except OSError:
                return
        body = json.dumps({"items": []}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    args, ros_args = ap.parse_known_args()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), _Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rclpy.init(args=ros_args)
    node = Node("standin")

    def accept(_request, response):
        response.accepted = True
        return response

    node.create_service(ReportFault, "/fault_manager/report_fault", accept)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
