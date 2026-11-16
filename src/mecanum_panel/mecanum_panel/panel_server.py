# Copyright 2026 Aryan Hegde
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

"""
Serve the booth demo's web pages: the panel, the TV and the development dashboard.

    ros2 run mecanum_panel panel_server --ros-args -p port:=8080 -p ws_port:=9090

A plain static file server over share/mecanum_panel/web (a ROS node only so that it starts and
stops with the launch). /config.js is generated: it tells the pages the websocket bridge's
port and whether to show the simulation controls. The pages connect to the bridge on the
host they were loaded from, so the same pages work on the laptop and from the robot.
"""

import functools
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Handler(SimpleHTTPRequestHandler):
    config_js = b''

    def do_GET(self):  # noqa: N802 (the base class's name)
        if self.path.split('?')[0] == '/config.js':
            self.send_response(200)
            self.send_header('Content-Type', 'text/javascript')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(self.config_js)
            return
        super().do_GET()

    def end_headers(self):
        # The pages change while developing; never serve a stale module.
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def log_message(self, fmt, *args):
        pass


class PanelServer(Node):

    def __init__(self):
        super().__init__('panel_server')
        p = self.declare_parameter
        port = p('port', 8080).value
        default_root = os.path.join(get_package_share_directory('mecanum_panel'), 'web')
        root = p('root', default_root).value
        config = {'wsPort': p('ws_port', 9090).value,
                  'simControls': p('sim_controls', False).value}
        handler = type('ConfiguredHandler', (Handler,), {
            'config_js': f'window.DEMO_CONFIG = {json.dumps(config)};\n'.encode()})
        self.httpd = ThreadingHTTPServer(('0.0.0.0', port),
                                         functools.partial(handler, directory=root))
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Serving {root} on http://localhost:{port}/ '
                               f'(panel /panel/, TV /tv/), bridge port {config["wsPort"]}')

    def shutdown(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = PanelServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
