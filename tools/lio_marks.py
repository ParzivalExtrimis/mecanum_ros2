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

r"""
Log the LIO odometry at tape marks on the floor, while the robot is pushed by hand (phase 4H).

There is no ground truth on hardware. Instead, the floor is marked with tape at measured
positions, the robot is pushed from mark to mark, and at each mark the operator presses Enter.
The tool records the /odom pose there, so it can be compared with the tape: for example, a
4.00 m straight push that LIO reports as 3.97 m, or a closed loop that ends 0.12 m from the
start mark.

While it runs, it prints a status line: the /odom pose, the distance travelled according to
/odom, the LIO's vertical drift (z of /lio/raw_odom, which should stay near 0 on a flat floor)
and the message rate. Type a label (or nothing) and press Enter to record a mark. Type q and
Enter, or press Ctrl-D, to finish and print the table.

/odom starts at (0, 0, 0) where LIO started, so start LIO with the robot standing on the start
mark, facing along the first leg.

Usage (with mecanum_bringup hardware_sensors.launch.py lio:=true running):
    tools/lio_marks.py --csv lio_marks_loop1.csv
Simulation (for a dry run):
    tools/lio_marks.py --use-sim-time
"""

import argparse
import math
import queue
import sys
import threading
import time


def yaw_from_quaternion(q):
    """Return the yaw (rad) of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def format_marks(marks):
    """Return the marks table as text. Each mark is (label, t, x, y, yaw, path, z)."""
    lines = [f'{"mark":<12} {"t [s]":>7} {"x [m]":>8} {"y [m]":>8} {"yaw [deg]":>10} '
             f'{"|xy| [m]":>9} {"path [m]":>9} {"lio z [m]":>10}']
    t0 = marks[0][1] if marks else 0.0
    for label, t, x, y, yaw, path, z in marks:
        lines.append(f'{label:<12} {t - t0:7.1f} {x:8.3f} {y:8.3f} {math.degrees(yaw):10.2f} '
                     f'{math.hypot(x, y):9.3f} {path:9.3f} {z:10.3f}')
    return '\n'.join(lines)


def main(argv=None):
    """Spin on /odom, record a mark on each Enter, print the table at the end."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--raw-topic', default='/lio/raw_odom',
                        help='3D LIO odometry, for the vertical drift')
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--csv', default='', help='also write the marks to this CSV file')
    args = parser.parse_args(argv)

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    rclpy.init()
    node = Node('lio_marks')
    if args.use_sim_time:
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    state = {'pose': None, 'path': 0.0, 'z': 0.0, 'count': 0, 'stamp': 0.0}

    def on_odom(m):
        p = m.pose.pose.position
        pose = (p.x, p.y, yaw_from_quaternion(m.pose.pose.orientation))
        if state['pose'] is not None:
            state['path'] += math.hypot(pose[0] - state['pose'][0], pose[1] - state['pose'][1])
        state['pose'] = pose
        state['count'] += 1
        state['stamp'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_raw(m):
        state['z'] = m.pose.pose.position.z

    node.create_subscription(Odometry, args.odom_topic, on_odom, 50)
    node.create_subscription(Odometry, args.raw_topic, on_raw, 50)

    lines = queue.Queue()

    def read_stdin():
        for line in sys.stdin:
            lines.put(line.strip())
        lines.put(None)

    threading.Thread(target=read_stdin, daemon=True).start()
    print(f'Listening on {args.odom_topic}. Type a label and press Enter at each mark; '
          'q + Enter (or Ctrl-D) to finish.', flush=True)

    marks = []
    last_status = 0.0
    last_count, last_count_time = 0, time.monotonic()
    rate = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_count_time >= 2.0:
                rate = (state['count'] - last_count) / (now - last_count_time)
                last_count, last_count_time = state['count'], now
            if now - last_status >= 0.5 and state['pose'] is not None:
                last_status = now
                x, y, yaw = state['pose']
                sys.stdout.write(
                    f'\rx {x:+7.3f}  y {y:+7.3f}  yaw {math.degrees(yaw):+7.2f} deg  '
                    f'path {state["path"]:7.2f} m  lio z {state["z"]:+6.3f} m  '
                    f'{rate:4.1f} Hz   ')
                sys.stdout.flush()
            try:
                line = lines.get_nowait()
            except queue.Empty:
                continue
            if line is None or line.lower() == 'q':
                break
            if state['pose'] is None:
                print(f'\nNo message on {args.odom_topic} yet; mark not recorded.', flush=True)
                continue
            label = line or f'mark{len(marks) + 1}'
            marks.append((label, state['stamp'], *state['pose'], state['path'], state['z']))
            x, y, yaw = state['pose']
            print(f'\n[{label}] x {x:.3f} y {y:.3f} yaw {math.degrees(yaw):.2f} deg, '
                  f'{math.hypot(x, y):.3f} m from the start', flush=True)
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: B902
        # SIGTERM shuts the context down under spin_once, which then raises RCLError.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print('\n' + (format_marks(marks) if marks else 'No marks recorded.'))
    if args.csv and marks:
        with open(args.csv, 'w') as f:
            f.write('label,t,x,y,yaw,path,lio_z\n')
            for label, t, x, y, yaw, path, z in marks:
                f.write(f'{label},{t:.3f},{x:.4f},{y:.4f},{yaw:.5f},{path:.4f},{z:.4f}\n')
        print(f'Wrote {args.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
