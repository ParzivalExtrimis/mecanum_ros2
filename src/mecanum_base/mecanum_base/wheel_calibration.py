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
Per-wheel calibration: wheel speed (rad/s) to ESP32 (direction, value), and back.

Open-loop model of one motor channel::

    value = deadband + scale * |omega|      (capped at max_value)
    direction = sign * sign(omega)

``deadband`` is the smallest value that makes the wheel turn, ``scale`` is value per rad/s
above the deadband, and ``sign`` flips a motor whose positive direction rolls the robot
backward. A wheel speed below ``zero_threshold`` is sent as a stop (direction 0, value 0).
"""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass(frozen=True)
class WheelCalibration:
    """Calibration of one wheel's motor channel on the ESP32."""

    motor_id: int
    sign: int
    scale: float
    deadband: float
    max_value: float
    zero_threshold: float = 1e-3

    def __post_init__(self):
        if self.sign not in (1, -1):
            raise ValueError(f'sign must be +1 or -1, got {self.sign!r}')
        if self.motor_id < 0:
            raise ValueError(f'motor_id must be >= 0, got {self.motor_id!r}')
        if not (math.isfinite(self.scale) and self.scale > 0.0):
            raise ValueError(f'scale must be positive, got {self.scale!r}')
        if not (math.isfinite(self.deadband) and self.deadband >= 0.0):
            raise ValueError(f'deadband must be >= 0, got {self.deadband!r}')
        if not (math.isfinite(self.max_value) and self.max_value > self.deadband):
            raise ValueError(
                f'max_value ({self.max_value!r}) must be greater than deadband '
                f'({self.deadband!r})')
        if not (self.zero_threshold >= 0.0):
            raise ValueError('zero_threshold must be >= 0')

    def max_omega(self) -> float:
        """Return the largest wheel speed (rad/s) reachable without exceeding max_value."""
        return (self.max_value - self.deadband) / self.scale

    def to_command(self, omega: float) -> Tuple[int, float]:
        """Convert a wheel speed (rad/s) to (direction, value). Non-finite input stops."""
        if not math.isfinite(omega) or abs(omega) < self.zero_threshold:
            return 0, 0.0
        direction = self.sign * (1 if omega > 0.0 else -1)
        value = min(self.deadband + self.scale * abs(omega), self.max_value)
        return direction, value

    def to_omega(self, direction: int, value: float) -> float:
        """Return the wheel speed (rad/s) that ``to_command`` maps to (direction, value)."""
        if direction == 0 or value <= self.deadband:
            return 0.0
        magnitude = (min(value, self.max_value) - self.deadband) / self.scale
        return self.sign * (1 if direction > 0 else -1) * magnitude
