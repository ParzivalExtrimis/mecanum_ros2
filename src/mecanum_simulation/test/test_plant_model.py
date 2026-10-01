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

"""Unit tests for mecanum_simulation.plant_model."""

from mecanum_simulation.plant_model import MotorPlant
import pytest


def test_ideal_plant_inverts_driver_calibration():
    # Driver stand-in calibration: value = omega / 15 -> plant gives omega back.
    plant = MotorPlant()
    assert plant.omega(1, 0.4) == pytest.approx(6.0)
    assert plant.omega(-1, 0.4) == pytest.approx(-6.0)


def test_deadband_and_stop():
    plant = MotorPlant(deadband=0.1)
    assert plant.omega(1, 0.05) == 0.0
    assert plant.omega(1, 0.1) == 0.0
    assert plant.omega(0, 0.8) == 0.0
    assert plant.omega(1, 0.2) == pytest.approx(0.1 * 15.0)


def test_saturates_at_max_omega():
    assert MotorPlant(max_omega=10.0).omega(1, 1.0) == pytest.approx(10.0)


def test_sign_models_reversed_motor():
    assert MotorPlant(sign=-1).omega(1, 0.4) == pytest.approx(-6.0)


def test_invalid_value_stops():
    assert MotorPlant().omega(1, float('nan')) == 0.0


@pytest.mark.parametrize('bad', [{'sign': 2}, {'scale': 0.0}, {'deadband': -1.0},
                                 {'max_omega': 0.0}])
def test_rejects_invalid_parameters(bad):
    with pytest.raises(ValueError):
        MotorPlant(**bad)
