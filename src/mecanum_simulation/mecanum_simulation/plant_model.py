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
Simulated motor channel: ESP32 (direction, value) -> wheel speed in rad/s.

This is the "true" plant the driver's calibration tries to invert. Making it differ from the
driver's calibration (scale, deadband, sign, wiring) lets the motion test and the calibration
tool be exercised against known errors::

    omega = sign * direction * (value - deadband) / scale    if value > deadband, else 0
    |omega| <= max_omega
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MotorPlant:
    """Open-loop response of one motor channel."""

    sign: int = 1
    scale: float = 1.0 / 15.0
    deadband: float = 0.0
    max_omega: float = 15.0

    def __post_init__(self):
        if self.sign not in (1, -1):
            raise ValueError(f'sign must be +1 or -1, got {self.sign!r}')
        if not (math.isfinite(self.scale) and self.scale > 0.0):
            raise ValueError(f'scale must be positive, got {self.scale!r}')
        if not (math.isfinite(self.deadband) and self.deadband >= 0.0):
            raise ValueError(f'deadband must be >= 0, got {self.deadband!r}')
        if not (math.isfinite(self.max_omega) and self.max_omega > 0.0):
            raise ValueError(f'max_omega must be positive, got {self.max_omega!r}')

    def omega(self, direction: int, value: float) -> float:
        """Return the wheel speed (rad/s) for an ESP32 command. Invalid input stops."""
        if direction == 0 or not math.isfinite(value) or value <= self.deadband:
            return 0.0
        magnitude = min((value - self.deadband) / self.scale, self.max_omega)
        return self.sign * (1 if direction > 0 else -1) * magnitude
