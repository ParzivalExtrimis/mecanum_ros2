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
The segment driver's control law, without ROS: one prewritten base move at a time.

A move is a straight holonomic line to (x, y) with the heading going linearly from the start
heading to the target heading along the way; a turn is a turn in place to a heading; a spin is
a turn in place by a signed angle that may exceed a full turn. Speeds follow a trapezoid:
limited by the configured speed, by the acceleration from the previous command, and by the
distance left (sqrt(2 a d)), so that the robot slows into the target. Off-line drift is pulled
back with a proportional term, and position is held during turns.

``step(pose, dt)`` takes the measured pose (map frame) and returns the body-frame command
(vx, vy, wz) and whether the move has arrived.
"""

import math

from mecanum_demo.geometry import wrap

POSITION_TOLERANCE = 0.015
YAW_TOLERANCE = 0.025
SETTLE_S = 0.2          # inside the tolerances for this long counts as arrived


class SegmentController:
    """Follows one segment. See the module docstring."""

    def __init__(self, kind, start, target=None, keep_heading=False, delta_yaw=0.0,
                 slow=False, speeds=None):
        self.kind = kind
        self.start = tuple(start)
        self.speeds = speeds
        self.v_max = speeds['linear_slow'] if slow else speeds['linear']
        self.w_max = speeds['angular'] * (0.6 if slow else 1.0)
        self.a_max = speeds['linear_accel']
        self.alpha_max = speeds['angular_accel']
        self.k_cross = 2.0
        self.k_yaw = 2.5
        self.k_hold = 1.5
        self.cmd = (0.0, 0.0, 0.0)      # the previous command, map-frame vx, vy and wz
        self.settled = 0.0
        if kind == 'move':
            self.goal = (target[0], target[1])
            dx, dy = self.goal[0] - start[0], self.goal[1] - start[1]
            self.length = math.hypot(dx, dy)
            self.unit = (dx / self.length, dy / self.length) if self.length > 1e-9 else (1, 0)
            self.yaw_start = start[2]
            self.yaw_change = 0.0 if keep_heading else wrap(target[2] - start[2])
        elif kind == 'turn':
            self.goal = (start[0], start[1])
            self.yaw_target = target[2]
        elif kind == 'spin':
            self.goal = (start[0], start[1])
            self.turned = 0.0           # unwrapped angle turned so far
            self.delta = delta_yaw
            self.last_yaw = start[2]
        else:
            raise ValueError(f'unknown segment kind "{kind}"')

    def progress(self, pose):
        """Return (fraction done 0..1, metres left, radians left)."""
        if self.kind == 'move':
            along = ((pose[0] - self.start[0]) * self.unit[0]
                     + (pose[1] - self.start[1]) * self.unit[1])
            left = math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])
            fraction = 1.0 if self.length < 1e-6 else min(max(along / self.length, 0.0), 1.0)
            yaw_left = abs(wrap(self._yaw_at(1.0) - pose[2]))
            return fraction, left, yaw_left
        if self.kind == 'turn':
            left = abs(wrap(self.yaw_target - pose[2]))
            total = abs(wrap(self.yaw_target - self.start[2])) or 1e-6
            return min(max(1.0 - left / total, 0.0), 1.0), 0.0, left
        left = abs(self.delta - self.turned)
        return min(max(1.0 - left / (abs(self.delta) or 1e-6), 0.0), 1.0), 0.0, left

    def remaining_s(self, pose):
        _, metres, radians = self.progress(pose)
        return metres / self.v_max + radians / self.w_max

    def _yaw_at(self, fraction):
        return self.yaw_start + self.yaw_change * fraction

    def _limit(self, wanted, previous, rate, dt):
        step = rate * dt
        return previous + max(-step, min(step, wanted - previous))

    def step(self, pose, dt):
        """Return ((vx, vy, wz) in the body frame, arrived)."""
        x, y, yaw = pose
        dt = max(dt, 1e-3)
        if self.kind == 'spin':
            self.turned += wrap(yaw - self.last_yaw)
            self.last_yaw = yaw

        # Translation, map frame.
        ex, ey = self.goal[0] - x, self.goal[1] - y
        if self.kind == 'move':
            along = ex * self.unit[0] + ey * self.unit[1]
            cross = (ex - along * self.unit[0], ey - along * self.unit[1])
            distance = math.hypot(ex, ey)
            speed = min(self.v_max, math.sqrt(2.0 * self.a_max * max(along, 0.0)),
                        1.5 * max(along, 0.0))
            if distance > POSITION_TOLERANCE:
                speed = max(speed, 0.02)
            vx = speed * self.unit[0] + self.k_cross * cross[0]
            vy = speed * self.unit[1] + self.k_cross * cross[1]
            if along < 0.0:      # overshot: come straight back
                vx, vy = self.k_hold * ex, self.k_hold * ey
            fraction, _, _ = self.progress(pose)
            yaw_error = wrap(self._yaw_at(min(fraction + 0.05, 1.0)) - yaw)
        else:
            distance = math.hypot(ex, ey)
            vx, vy = self.k_hold * ex, self.k_hold * ey
            if self.kind == 'turn':
                yaw_error = wrap(self.yaw_target - yaw)
            else:
                yaw_error = self.delta - self.turned
        norm = math.hypot(vx, vy)
        if norm > self.v_max:
            vx, vy = vx * self.v_max / norm, vy * self.v_max / norm

        # Rotation.
        if self.kind == 'move':
            wz = max(-self.w_max, min(self.w_max, self.k_yaw * yaw_error))
        else:
            magnitude = min(self.w_max, math.sqrt(2.0 * self.alpha_max * abs(yaw_error)),
                            self.k_yaw * abs(yaw_error))
            if abs(yaw_error) > YAW_TOLERANCE:
                magnitude = max(magnitude, 0.05)
            wz = math.copysign(magnitude, yaw_error)

        # Acceleration limits, then the body frame.
        vx = self._limit(vx, self.cmd[0], self.a_max, dt)
        vy = self._limit(vy, self.cmd[1], self.a_max, dt)
        wz = self._limit(wz, self.cmd[2], self.alpha_max, dt)
        self.cmd = (vx, vy, wz)

        arrived_now = distance <= POSITION_TOLERANCE and abs(
            wrap(self._final_yaw() - yaw) if self.kind != 'spin' else yaw_error) <= YAW_TOLERANCE
        self.settled = self.settled + dt if arrived_now else 0.0
        if self.settled >= SETTLE_S:
            self.cmd = (0.0, 0.0, 0.0)
            return (0.0, 0.0, 0.0), True
        if arrived_now:
            return (0.0, 0.0, 0.0), False
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * vx + s * vy, -s * vx + c * vy, wz), False

    def _final_yaw(self):
        if self.kind == 'move':
            return self._yaw_at(1.0)
        if self.kind == 'turn':
            return self.yaw_target
        return self.start[2] + self.delta
