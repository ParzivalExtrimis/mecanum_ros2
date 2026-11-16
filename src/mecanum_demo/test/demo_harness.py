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

"""Shared helpers of the mecanum_demo tests: the site config and a driver for DemoLogic."""

import math
import os

from mecanum_demo.demo_logic import DemoLogic
from mecanum_demo.geometry import wrap
import yaml

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'config', 'booth_demo.yaml')


def load_config():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


class Harness:
    """A DemoLogic with a base that reaches every requested pose at once."""

    def __init__(self, at='home', heading='W'):
        self.cfg = load_config()
        self.logic = DemoLogic(self.cfg)
        x, y = self.cfg['poses'][at]
        self.logic.set_pose((x, y, self.cfg['headings'][heading]))
        self.moves = []      # labels of the base moves carried out

    def cmd(self, action, *args):
        return self.logic.command(action, args)

    def ok(self, action, *args):
        accepted, reason = self.logic.command(action, args)
        assert accepted, f'{action} refused: {reason}'

    def finish_motion(self, success=True, error=''):
        """Carry out the pending base move (teleport) and report its result."""
        motion = self.logic.motion
        assert motion is not None
        if success:
            x, y, yaw = self.logic.pose
            if motion['kind'] == 'move':
                x, y = motion['x'], motion['y']
                if not motion['keep_heading']:
                    yaw = motion['yaw']
            elif motion['kind'] == 'turn':
                yaw = motion['yaw']
            else:
                yaw = wrap(yaw + motion['delta_yaw'])
            self.logic.set_pose((x, y, yaw))
            self.moves.append(motion['label'])
        self.logic.segment_result(motion['id'], success, error)

    def run(self, until, limit=400, dt=0.5):
        """Tick and finish base moves until ``until(logic)`` holds."""
        for _ in range(limit):
            if until(self.logic):
                return
            if self.logic.motion is not None and self.logic.state == 'running':
                self.finish_motion()
            else:
                self.logic.tick(dt)
        raise AssertionError(f'not reached; state {self.logic.state}, '
                             f'feature {self.logic.feat}, step {self.logic.si}')

    def run_until_state(self, state, **kwargs):
        self.run(lambda logic: logic.state == state, **kwargs)

    def run_until_feature(self, name, **kwargs):
        self.run(lambda logic: logic.feat == name and logic.state == 'running', **kwargs)

    def at(self, spot):
        x, y = self.cfg['poses'][spot]
        return math.hypot(self.logic.pose[0] - x, self.logic.pose[1] - y) < 1e-6
