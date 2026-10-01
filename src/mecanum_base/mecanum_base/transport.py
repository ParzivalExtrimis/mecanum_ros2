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
Transports that carry per-motor commands from the driver to the ESP32.

- ``mock``: logs commands, for dry runs without hardware or a simulator.
- ``sim``: publishes ``mecanum_interfaces/MotorCommands`` for the simulated ESP32
  (``mecanum_simulation/sim_motor_plant``).
- ``esp32``: the real network link. Not implemented until the rewritten firmware's protocol
  is fixed; selecting it fails at startup with an explanation.
"""

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Optional, Sequence, Tuple


@dataclass(frozen=True)
class MotorCommand:
    """One motor channel command: ESP32 motor address, direction (+1/-1/0) and value."""

    motor_id: int
    direction: int
    value: float


class Transport(ABC):
    """Sends motor commands: ``connect`` once, ``send`` at a fixed rate, ``close`` once."""

    def connect(self) -> None:  # noqa: B027 - optional hook, intentionally empty
        """Open the link. Called once before the first ``send``."""

    @abstractmethod
    def send(self, commands: Sequence[MotorCommand]) -> None:
        """Send one command per motor. Must not block for longer than one control period."""

    def close(self) -> None:  # noqa: B027 - optional hook, intentionally empty
        """Close the link. Called once, after the final zero command has been sent."""


class MockTransport(Transport):
    """Logs commands instead of sending them, and keeps a short history for tests."""

    def __init__(self, log: Optional[Callable[[str], None]] = None,
                 log_every: int = 20, history: int = 100):
        self._log = log or print
        self._log_every = max(1, int(log_every))
        self._count = 0
        self._last: Optional[Tuple[MotorCommand, ...]] = None
        self.sent: Deque[Tuple[MotorCommand, ...]] = deque(maxlen=history)

    def send(self, commands: Sequence[MotorCommand]) -> None:
        commands = tuple(commands)
        self.sent.append(commands)
        changed = commands != self._last
        if changed or self._count % self._log_every == 0:
            text = '  '.join(
                f'm{c.motor_id}:{c.direction:+d}/{c.value:.3f}' for c in commands)
            self._log(f'[mock esp32] {text}')
        self._last = commands
        self._count += 1


class SimTransport(Transport):
    """Publishes commands as ``mecanum_interfaces/MotorCommands`` for the simulated ESP32."""

    def __init__(self, node, topic: str = '/esp32/motor_commands'):
        from mecanum_interfaces.msg import MotorCommands
        self._msg_type = MotorCommands
        self._node = node
        self._topic = topic
        self._pub = None

    def connect(self) -> None:
        self._pub = self._node.create_publisher(self._msg_type, self._topic, 10)

    def send(self, commands: Sequence[MotorCommand]) -> None:
        if self._pub is None:
            raise RuntimeError('SimTransport.send() called before connect()')
        msg = self._msg_type()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.motor_id = [int(c.motor_id) for c in commands]
        msg.direction = [int(c.direction) for c in commands]
        msg.value = [float(c.value) for c in commands]
        self._pub.publish(msg)

    def close(self) -> None:
        if self._pub is not None:
            self._node.destroy_publisher(self._pub)
            self._pub = None


class Esp32Transport(Transport):
    """Real ESP32 link. Placeholder until the firmware protocol is defined."""

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(
            'The ESP32 transport is not implemented yet: the motor firmware is being rewritten '
            'and its protocol (transport, address/port, message format, value units, '
            'direction encoding, motor IDs, watchdog) is not defined. Use transport:=mock or '
            'transport:=sim. See src/mecanum_base/README.md, "Real ESP32 transport".')

    def send(self, commands: Sequence[MotorCommand]) -> None:  # pragma: no cover
        raise NotImplementedError


TRANSPORTS = ('mock', 'sim', 'esp32')


def make_transport(name: str, node=None, log: Optional[Callable[[str], None]] = None,
                   sim_topic: str = '/esp32/motor_commands') -> Transport:
    """Create a transport by name: 'mock', 'sim' or 'esp32'."""
    if name == 'mock':
        return MockTransport(log=log)
    if name == 'sim':
        if node is None:
            raise ValueError('the sim transport needs a ROS node')
        return SimTransport(node, sim_topic)
    if name == 'esp32':
        return Esp32Transport()
    raise ValueError(f'unknown transport {name!r}, expected one of {TRANSPORTS}')
