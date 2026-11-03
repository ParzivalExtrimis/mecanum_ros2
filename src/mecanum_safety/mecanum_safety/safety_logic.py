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
The safety rules of docs/plan.md Phase 4, without ROS.

One rule covers every directional block. A **guard** is a set of robot points, each with
outward normals. A command (vx, vy, wz) in base_footprint passes a guard only if every point
moves with a non-positive speed along every normal, so the guarded part of the robot does not
move toward its hazard. The speed of a point (x, y) is (vx - wz * y, vy + wz * x), so rotation,
which sweeps the corners, is judged exactly.

- A cliff, or a lost cliff sensor, guards its corner, in both outward axes: the direction of the
  edge is unknown.
- A pressed plank, or a lost bumper, guards the plank's two ends, along the plank's normal.
- A lost ultrasonic guards its face of the robot, along the face's normal.
- The e-stop and a lost full-stop sensor (scan, L2, D435i, joint states, odometry) stop all
  motion.

A command that fails any guard is replaced with zero, not reduced: the gate never invents a
motion that no source asked for.
"""

from dataclasses import dataclass, field
import math

EPSILON = 1e-9


@dataclass(frozen=True)
class Twist2D:
    """A planar body twist in base_footprint: m/s, m/s, rad/s."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def is_zero(self):
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


ZERO = Twist2D()


@dataclass(frozen=True)
class Guard:
    """Robot points (x, y) in base_footprint and the outward unit normals they may not follow."""

    points: tuple
    normals: tuple

    def allows(self, cmd):
        for x, y in self.points:
            px, py = cmd.vx - cmd.wz * y, cmd.vy + cmd.wz * x
            for nx, ny in self.normals:
                if px * nx + py * ny > EPSILON:
                    return False
        return True


def corner_guard(x, y):
    """Guard a corner point in both of its outward axes."""
    return Guard(points=((x, y),),
                 normals=((math.copysign(1.0, x), 0.0), (0.0, math.copysign(1.0, y))))


def face_guard(x, half_width):
    """Guard a front (x > 0) or rear (x < 0) face, through its two ends, along its normal."""
    return Guard(points=((x, half_width), (x, -half_width)),
                 normals=((math.copysign(1.0, x), 0.0),))


def is_cliff(range_m, floor_distance, margin):
    """
    Return whether a downward range shows no floor.

    REP 117: +inf is no return (no floor in reach), -inf an object nearer than the minimum
    range, which is floor. NaN is an invalid reading and counts as a cliff.
    """
    if math.isnan(range_m):
        return True
    return range_m > floor_distance + margin


@dataclass
class Watchdog:
    """
    Topics that must keep arriving, each with its timeout in seconds.

    A topic counts as lost until its first message, and again once its last message is older
    than its timeout. Time is whatever monotonic clock the caller passes in.
    """

    timeouts: dict
    last: dict = field(default_factory=dict)

    def feed(self, topic, now):
        if topic in self.timeouts:
            self.last[topic] = now

    def lost(self, now):
        """Return the lost topics, sorted."""
        return sorted(topic for topic, timeout in self.timeouts.items()
                      if topic not in self.last or now - self.last[topic] > timeout)


@dataclass
class EStop:
    """An e-stop that latches on true and clears only on a reset while the input is false."""

    active: bool = False
    latched: bool = False

    def update(self, value):
        self.active = bool(value)
        if self.active:
            self.latched = True

    def reset(self):
        """Clear the latch; return (success, message)."""
        if self.active:
            return False, 'The e-stop is still pressed (/estop is true)'
        was = self.latched
        self.latched = False
        return True, 'E-stop reset' if was else 'The e-stop was not latched'


@dataclass(frozen=True)
class Decision:
    """What the supervisor blocks: a full stop, or the guards that apply, with the reasons."""

    stopped: bool
    guards: tuple
    reasons: tuple

    def gate(self, cmd):
        """Return the command if every block allows it, else zero."""
        if self.stopped:
            return ZERO
        if all(guard.allows(cmd) for guard in self.guards):
            return cmd
        return ZERO


@dataclass(frozen=True)
class Layout:
    """
    The guarded parts of the robot, in base_footprint (from mecanum_description).

    cliffs: position name -> (x, y) of the downward sensor.
    bumpers: 'front' / 'rear' -> plank x; the plank spans +-half_width.
    ultrasonics: position name -> x of the face it watches ('front_left' -> the front face).
    """

    cliffs: dict
    bumpers: dict
    ultrasonics: dict
    half_width: float


def decide(layout, *, estop_latched, lost, full_stop_topics, cliff_ranges, floor_distance,
           cliff_margin, bumpers_pressed, topic_of):
    """
    Combine every input into a Decision.

    lost: the lost topics, from Watchdog.lost.
    full_stop_topics: the topics whose loss stops all motion.
    cliff_ranges: position -> latest range (absent: none yet; the watchdog reports it lost).
    bumpers_pressed: plank -> latest state.
    topic_of: maps ('cliff' | 'bumper' | 'ultrasonic', name) to its topic, to match lost topics.
    """
    reasons = []
    guards = []
    lost = set(lost)
    stopped = False
    if estop_latched:
        stopped = True
        reasons.append('estop')
    for topic in sorted(lost & set(full_stop_topics)):
        stopped = True
        reasons.append(f'lost {topic}')
    for name, (x, y) in layout.cliffs.items():
        topic = topic_of('cliff', name)
        if topic in lost:
            guards.append(corner_guard(x, y))
            reasons.append(f'lost {topic}')
        elif name in cliff_ranges and is_cliff(cliff_ranges[name], floor_distance, cliff_margin):
            guards.append(corner_guard(x, y))
            reasons.append(f'cliff {name}')
    for name, x in layout.bumpers.items():
        topic = topic_of('bumper', name)
        if topic in lost:
            guards.append(face_guard(x, layout.half_width))
            reasons.append(f'lost {topic}')
        elif bumpers_pressed.get(name, False):
            guards.append(face_guard(x, layout.half_width))
            reasons.append(f'bumper {name}')
    for name, x in layout.ultrasonics.items():
        topic = topic_of('ultrasonic', name)
        if topic in lost:
            guards.append(face_guard(x, layout.half_width))
            reasons.append(f'lost {topic}')
    return Decision(stopped=stopped, guards=tuple(guards), reasons=tuple(sorted(reasons)))
