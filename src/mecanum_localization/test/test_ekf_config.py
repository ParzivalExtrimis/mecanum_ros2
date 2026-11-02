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

"""Check the EKF candidates: their inputs, frames and the single owner of the TF."""

import os

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config')
CONTROLLERS = os.path.join(HERE, '..', '..', 'mecanum_controllers', 'config', 'controllers.yaml')

# Positions in robot_localization's 15-element config vectors.
X, Y, YAW, VX, VY, VYAW = 0, 1, 5, 6, 7, 11
CANDIDATES = ('ekf.yaml', 'ekf_e2.yaml', 'ekf_e3a.yaml', 'ekf_e3b.yaml')


def load(name):
    with open(os.path.join(CONFIG, name)) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def used(params, key):
    """Return the indices a config vector enables."""
    vector = params[f'{key}_config']
    assert len(vector) == 15
    return {i for i, on in enumerate(vector) if on}


def added(name):
    """Return the parameters a candidate has that E1 lacks."""
    base = load('ekf.yaml')
    params = load(name)
    return {k: v for k, v in params.items() if k not in base}


@pytest.mark.parametrize('name', CANDIDATES)
def test_frames_and_mode(name):
    params = load(name)
    assert params['two_d_mode'] is True
    assert params['frequency'] == 50.0
    assert params['world_frame'] == params['odom_frame'] == 'odom'
    assert params['base_link_frame'] == 'base_footprint'


@pytest.mark.parametrize('name', CANDIDATES)
def test_wheels_give_twist_only(name):
    params = load(name)
    assert params['odom0'] == '/wheel/odometry'
    assert used(params, 'odom0') == {VX, VY, VYAW}


@pytest.mark.parametrize('name', CANDIDATES)
def test_l2_gyro_gives_yaw_rate_only(name):
    params = load(name)
    assert params['imu0'] == '/l2/imu'
    assert used(params, 'imu0') == {VYAW}


def test_imu_queue_holds_a_filter_cycle():
    # 250 Hz into a 50 Hz filter is 5 samples per cycle.
    assert load('ekf.yaml')['imu0_queue_size'] > 5


def test_only_the_default_publishes_tf():
    assert load('ekf.yaml')['publish_tf'] is True
    for name in CANDIDATES[1:]:
        assert load(name)['publish_tf'] is False


def test_the_drive_controller_leaves_the_tf_to_the_ekf():
    with open(CONTROLLERS) as f:
        params = yaml.safe_load(f)['mecanum_drive_controller']['ros__parameters']
    assert params['enable_odom_tf'] is False


def test_e2_adds_the_d435i_gyro():
    params = load('ekf_e2.yaml')
    assert params['imu1'] == '/d435i/imu'
    assert used(params, 'imu1') == {VYAW}
    assert set(added('ekf_e2.yaml')) == {'imu1', 'imu1_config', 'imu1_queue_size'}


def test_e3a_adds_the_lio_twist():
    params = load('ekf_e3a.yaml')
    assert params['odom1'] == '/lio/odom'
    assert used(params, 'odom1') == {VX, VY, VYAW}
    assert set(added('ekf_e3a.yaml')) == {'odom1', 'odom1_config', 'odom1_queue_size'}


def test_e3b_adds_the_lio_pose_as_a_differential():
    params = load('ekf_e3b.yaml')
    assert params['odom1'] == '/lio/odom'
    assert used(params, 'odom1') == {X, Y, YAW}
    assert params['odom1_differential'] is True
    assert set(added('ekf_e3b.yaml')) == {
        'odom1', 'odom1_config', 'odom1_queue_size', 'odom1_differential'}
