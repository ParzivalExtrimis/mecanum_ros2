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

"""Unit tests for mecanum_base.transport."""

from mecanum_base.transport import (Esp32Transport, make_transport, MockTransport,
                                    MotorCommand)
import pytest


def test_mock_transport_records_and_logs_changes():
    lines = []
    t = make_transport('mock', log=lines.append)
    assert isinstance(t, MockTransport)
    t.connect()
    cmds = [MotorCommand(i, 1, 0.5) for i in range(4)]
    t.send(cmds)
    t.send(cmds)
    t.send([MotorCommand(i, 0, 0.0) for i in range(4)])
    t.close()
    assert len(t.sent) == 3
    assert len(lines) == 2  # first send and the change; the repeat is not logged
    assert 'm0:+1/0.500' in lines[0]


def test_esp32_transport_is_not_implemented():
    with pytest.raises(NotImplementedError):
        Esp32Transport()
    with pytest.raises(NotImplementedError):
        make_transport('esp32')


def test_sim_transport_requires_node():
    with pytest.raises(ValueError):
        make_transport('sim')


def test_unknown_transport_rejected():
    with pytest.raises(ValueError):
        make_transport('carrier_pigeon')
