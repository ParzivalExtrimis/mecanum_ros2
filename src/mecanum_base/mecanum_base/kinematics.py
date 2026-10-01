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
Mecanum kinematics for an X-roller layout.

Pure Python with no ROS imports, so it can be unit-tested and reused anywhere.

Conventions (REP-103): body frame x forward, y left, z up. Wheel speeds are in rad/s and are
positive when the wheel rolls the robot forward. Wheel order everywhere in this package is
``WHEEL_NAMES``: front_left, front_right, rear_left, rear_right.

Inverse kinematics, with lx = half wheelbase, ly = half track, r = wheel radius::

    FL = (vx - vy - (lx + ly) * wz) / r
    FR = (vx + vy + (lx + ly) * wz) / r
    RL = (vx + vy - (lx + ly) * wz) / r
    RR = (vx - vy + (lx + ly) * wz) / r

These assume the rotation centre is the geometric centre of the four wheels.
"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

WHEEL_NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')

Twist2D = Tuple[float, float, float]
WheelSpeeds = Tuple[float, float, float, float]


@dataclass(frozen=True)
class MecanumGeometry:
    """Wheel geometry: half wheelbase lx, half track ly and wheel radius, all in metres."""

    lx: float
    ly: float
    wheel_radius: float

    def __post_init__(self):
        for name in ('lx', 'ly', 'wheel_radius'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be a positive finite number, got {value!r}')

    @property
    def k(self) -> float:
        """Return lx + ly, the lever arm used for rotation."""
        return self.lx + self.ly


def inverse(vx: float, vy: float, wz: float, geometry: MecanumGeometry) -> WheelSpeeds:
    """Return wheel speeds (rad/s) for a body twist (m/s, m/s, rad/s)."""
    k = geometry.k
    r = geometry.wheel_radius
    return (
        (vx - vy - k * wz) / r,
        (vx + vy + k * wz) / r,
        (vx + vy - k * wz) / r,
        (vx - vy + k * wz) / r,
    )


def forward(wheels: Sequence[float], geometry: MecanumGeometry) -> Twist2D:
    """Return the body twist (m/s, m/s, rad/s) for four wheel speeds (rad/s)."""
    fl, fr, rl, rr = wheels
    r = geometry.wheel_radius
    vx = r * (fl + fr + rl + rr) / 4.0
    vy = r * (-fl + fr + rl - rr) / 4.0
    wz = r * (-fl + fr - rl + rr) / (4.0 * geometry.k)
    return (vx, vy, wz)


def clamp_twist(vx: float, vy: float, wz: float,
                max_vx: float, max_vy: float, max_wz: float) -> Tuple[Twist2D, float]:
    """
    Scale a twist down uniformly so that every axis is within its limit.

    Uniform scaling keeps the direction of motion and the path curvature, unlike clamping
    each axis on its own. Returns the scaled twist and the factor applied (1.0 = unchanged).
    """
    factor = 1.0
    for value, limit in ((vx, max_vx), (vy, max_vy), (wz, max_wz)):
        if limit <= 0.0:
            factor = 0.0
            break
        if abs(value) > limit:
            factor = min(factor, limit / abs(value))
    return (vx * factor, vy * factor, wz * factor), factor


def scale_to_limits(wheels: Sequence[float],
                    limits: Sequence[float]) -> Tuple[WheelSpeeds, float]:
    """
    Scale all wheel speeds by one common factor so that no wheel exceeds its limit.

    ``limits`` holds the maximum achievable |speed| of each wheel (rad/s). Using one factor
    for all four wheels preserves the direction of motion. Returns the scaled speeds and
    the factor applied (1.0 = unchanged).
    """
    if len(wheels) != len(limits):
        raise ValueError('wheels and limits must have the same length')
    factor = 1.0
    for speed, limit in zip(wheels, limits):
        if limit <= 0.0:
            if speed != 0.0:
                factor = 0.0
            continue
        if abs(speed) > limit:
            factor = min(factor, limit / abs(speed))
    return tuple(speed * factor for speed in wheels), factor
