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
ROS-free drive logic: body twist + time -> per-motor commands.

Pipeline, run once per control period::

    latest twist (or zero if older than cmd_timeout)
      -> clamp to velocity limits (uniform scaling)
      -> inverse kinematics
      -> proportional saturation scaling against each wheel's reachable speed
      -> per-wheel calibration -> (motor_id, direction, value)
"""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple

from mecanum_base.kinematics import (clamp_twist, inverse, MecanumGeometry,
                                     scale_to_limits, WHEEL_NAMES)
from mecanum_base.transport import MotorCommand
from mecanum_base.wheel_calibration import WheelCalibration


@dataclass(frozen=True)
class DriveLimits:
    """Body velocity limits: m/s, m/s, rad/s."""

    max_vx: float
    max_vy: float
    max_wz: float


@dataclass(frozen=True)
class DriveOutput:
    """Everything computed in one control period, in ``WHEEL_NAMES`` order."""

    commands: Tuple[MotorCommand, ...]
    twist: Tuple[float, float, float]
    omega_requested: Tuple[float, ...]
    omega_sent: Tuple[float, ...]
    saturation_factor: float
    timed_out: bool


class DriveCore:
    """Holds the latest velocity command and turns it into motor commands."""

    def __init__(self, geometry: MecanumGeometry,
                 calibrations: Dict[str, WheelCalibration],
                 limits: DriveLimits, cmd_timeout: float):
        missing = [name for name in WHEEL_NAMES if name not in calibrations]
        if missing:
            raise ValueError(f'missing calibration for wheels: {missing}')
        ids = [calibrations[name].motor_id for name in WHEEL_NAMES]
        if len(set(ids)) != len(ids):
            raise ValueError(f'motor_id values must be unique, got {ids}')
        if not cmd_timeout > 0.0:
            raise ValueError('cmd_timeout must be > 0')
        self.geometry = geometry
        self.calibrations = tuple(calibrations[name] for name in WHEEL_NAMES)
        self.limits = limits
        self.cmd_timeout = cmd_timeout
        self._twist: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._stamp: Optional[float] = None

    def set_command(self, vx: float, vy: float, wz: float, stamp: float) -> bool:
        """
        Store a velocity command received at time ``stamp`` (seconds).

        Returns False and ignores the command if any component is not finite.
        """
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            return False
        self._twist = (vx, vy, wz)
        self._stamp = stamp
        return True

    def is_timed_out(self, now: float) -> bool:
        """
        Return True if no command is fresh at time ``now``.

        A command stamped in the future (e.g. after a simulation clock reset) also
        counts as timed out.
        """
        if self._stamp is None:
            return True
        age = now - self._stamp
        return age < 0.0 or age > self.cmd_timeout

    def compute(self, now: float) -> DriveOutput:
        """Compute the motor commands to send at time ``now`` (seconds)."""
        timed_out = self.is_timed_out(now)
        vx, vy, wz = (0.0, 0.0, 0.0) if timed_out else self._twist
        (vx, vy, wz), _ = clamp_twist(
            vx, vy, wz, self.limits.max_vx, self.limits.max_vy, self.limits.max_wz)
        requested = inverse(vx, vy, wz, self.geometry)
        sent, factor = scale_to_limits(
            requested, [cal.max_omega() for cal in self.calibrations])
        commands = []
        for cal, omega in zip(self.calibrations, sent):
            direction, value = cal.to_command(omega)
            commands.append(MotorCommand(cal.motor_id, direction, value))
        return DriveOutput(tuple(commands), (vx, vy, wz), tuple(requested), tuple(sent),
                           factor, timed_out)

    def zero_output(self) -> DriveOutput:
        """Return an all-stop output, used on shutdown and after errors."""
        zeros = (0.0,) * len(self.calibrations)
        commands = tuple(MotorCommand(cal.motor_id, 0, 0.0) for cal in self.calibrations)
        return DriveOutput(commands, (0.0, 0.0, 0.0), zeros, zeros, 1.0, True)
