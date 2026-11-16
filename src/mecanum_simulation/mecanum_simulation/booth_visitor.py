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
The booth's visitor, moved in and out of the booth on request (sim shortcut S12).

    ros2 run mecanum_simulation booth_visitor

Serves /sim/visitor (std_srvs/SetBool): true moves the `visitor` model of worlds/booth.sdf to
the booth front, where it leans in at the robot's centre stage; false parks it at the end of
the aisle. The current place is on /sim/visitor_in (std_msgs/Bool, transient local). The move
goes through Gazebo's /world/<world>/set_pose service with the gz command-line tool, so no
bridge entry is needed. On hardware visitors come by themselves; nothing here runs there.
"""

import subprocess
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class BoothVisitor(Node):

    def __init__(self):
        super().__init__('booth_visitor')
        p = self.declare_parameter
        self.world = p('world', 'booth').value
        self.model = p('model', 'visitor').value
        # In front of centre stage, 0.40 m from the robot's side there (hold radius 0.45 m).
        self.inside = p('inside', [0.45, -1.15, 0.651]).value
        self.outside = p('outside', [3.5, -3.0, 0.651]).value
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Bool, '/sim/visitor_in', latched)
        self.pub.publish(Bool(data=False))
        self.lock = threading.Lock()
        self.create_service(SetBool, '/sim/visitor', self._on_request)

    def _on_request(self, request, response):
        x, y, z = self.inside if request.data else self.outside
        req = f'name: "{self.model}" position: {{x: {x} y: {y} z: {z}}}'
        with self.lock:
            try:
                out = subprocess.run(
                    ['gz', 'service', '-s', f'/world/{self.world}/set_pose',
                     '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', req],
                    capture_output=True, text=True, timeout=6.0)
                ok = 'data: true' in out.stdout
                detail = out.stdout.strip() or out.stderr.strip()
            except (OSError, subprocess.TimeoutExpired) as e:
                ok, detail = False, str(e)
        response.success = ok
        if ok:
            self.pub.publish(Bool(data=request.data))
            response.message = ('Visitor leans into the booth' if request.data
                                else 'Visitor steps back')
        else:
            response.message = f'Gazebo did not move the visitor: {detail}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BoothVisitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
