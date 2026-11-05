#!/usr/bin/env python3
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
Save slam_toolbox's map: the image and YAML for map_server, and the pose graph (Phase 5).

While mapping (mecanum_bringup mapping.launch.py, or slam_mapping.launch.py):
    tools/save_map.py restroom

writes, by default into src/mecanum_navigation/maps/ of this workspace:
    restroom.pgm, restroom.yaml          /slam_toolbox/save_map (map_server's map_saver)
    restroom.posegraph, restroom.data    /slam_toolbox/serialize_map (for localization mode
                                         and for continuing the map later)

slam_toolbox writes the files itself, so the paths passed to it are absolute.
"""

import argparse
import os
import sys

import rclpy
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String

MAPS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src',
                    'mecanum_navigation', 'maps')


def call(node, client, request, timeout):
    if not client.wait_for_service(timeout_sec=timeout):
        sys.exit(f'{client.srv_name} is not available: is slam_toolbox running?')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        sys.exit(f'{client.srv_name} did not answer within {timeout} s')
    return future.result()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('name', help='Map name, without extension')
    parser.add_argument('--dir', default=MAPS, help='Output directory (default: %(default)s)')
    parser.add_argument('--timeout', type=float, default=30.0, help='Seconds per service call')
    args = parser.parse_args(argv)
    os.makedirs(args.dir, exist_ok=True)
    base = os.path.join(os.path.abspath(args.dir), args.name)

    rclpy.init()
    node = rclpy.create_node('save_map')
    try:
        save = call(node, node.create_client(SaveMap, '/slam_toolbox/save_map'),
                    SaveMap.Request(name=String(data=base)), args.timeout)
        if save.result != SaveMap.Response.RESULT_SUCCESS:
            sys.exit(f'save_map failed (result {save.result})')
        graph = call(node, node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map'),
                     SerializePoseGraph.Request(filename=base), args.timeout)
        if graph.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            sys.exit(f'serialize_map failed (result {graph.result})')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    missing = [ext for ext in ('.pgm', '.yaml', '.posegraph', '.data')
               if not os.path.exists(base + ext)]
    if missing:
        sys.exit(f'not written: {", ".join(base + ext for ext in missing)}')
    for ext in ('.pgm', '.yaml', '.posegraph', '.data'):
        print(base + ext)


if __name__ == '__main__':
    main()
