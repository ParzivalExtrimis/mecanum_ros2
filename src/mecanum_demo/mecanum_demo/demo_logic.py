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
The booth demo's supervisor logic: one state machine that decides everything.

Pure Python, no ROS: demo_supervisor feeds it the robot's pose, whether someone is close, the
e-stop, the commands and the results of base moves, calls tick() with the time, and publishes
snapshot(). It is a port of the supervisor in docs/Booth demo simulator.html.

States
    idle      waiting for a mode or a single feature
    running   a feature's steps are being executed
    hold      someone is close during a base move; resumes hold_clear_s after the area clears
    gate      the teleop gate: nothing moves until START (Mode 4: falls back to the replay)
    live      a teleop session
    stowing   the session ended and the arm is stowing, then: ended
    ended     waiting for save or discard
    stopping  stopped with the arm out; the arm stows, then: stopped
    stopped   by staff, a fault or a released e-stop; waits for resume, go home, reset, manual
    manual    staff drive the base (sub "drive") or open the teleop gate (sub "arm")
    estop     the hardware e-stop is pressed

Base moves
    The logic never drives. ``motion`` is the base move it wants right now, or None, with an
    ``id`` that changes for every new request. The node sends it to the segment driver, cancels
    the running one when ``motion`` is None or changes, and reports the end with
    segment_result(id, ...).

The arm
    The arm is not part of this workspace. Arm steps are timed placeholders: the step is shown,
    its duration passes, the playlist goes on, and ``notice`` says that nothing moved. The
    teleop gate's three arm checks fail until arm_checks_pass is set (a development switch), so
    that the live and ended screens can still be shown.
"""

import math

from mecanum_demo.geometry import wrap

ARM_NOTICE = 'Arm action "{label}": the arm is not integrated yet, nothing moves.'
ARM_DETAIL = 'Arm not integrated'
MOVING_STATES = ('running', 'hold')


class DemoLogic:
    """The supervisor's state machine. See the module docstring."""

    def __init__(self, config):
        self.cfg = config
        self.features = config['features']
        self.modes = config['modes']
        self.poses = {k: tuple(v) for k, v in config['poses'].items()}
        self.headings = config['headings']
        self.speeds = config['speeds']

        self.state = 'idle'
        self.mode = None            # '1'..'5', 'single' or None
        self.playlist = []
        self.pi = 0
        self.loop_start = 0
        self.loop = False
        self.runs = 1
        self.feat = None
        self.segs = []
        self.si = 0
        self.message = ''
        self.stop_kind = ''
        self.origin = None          # the state a stop came from
        self.manual_sub = 'drive'
        self.manual_checks = False  # Manual with the safety checks on; off on every entry
        self.hold_check = 'on'      # on | off_loop (until this loop ends) | off (until toggled)
        self.gate_t = 0.0
        self.live_t = 0.0
        self.live_idle = 0.0
        self.ended_reason = ''
        self.ended_length = 0.0
        self.stow_t = 0.0
        self.stow_next = 'stopped'
        self.clear_t = 0.0
        self.demos = int(config.get('demo_count', 0))
        self.last_demo = self.demos
        self.notice = ''

        self.pose = None            # (x, y, yaw) in the map frame, None when unknown
        self.person_close = False
        self.estop = False
        self.arm_stowed = True
        self.arm_checks_pass = False
        self.base_moving = False

        self.motion = None
        self._motion_id = 0
        self.seg_remaining = None   # the driver's estimate for the running base move
        self.log = []               # (text) lines for the node to print; it empties the list

    # ----- inputs -------------------------------------------------------------------------

    def set_pose(self, pose):
        self.pose = pose

    def set_base_moving(self, moving):
        self.base_moving = bool(moving)

    def set_person_close(self, close):
        self.person_close = bool(close)

    def set_arm_checks_pass(self, value):
        self.arm_checks_pass = bool(value)

    def set_segment_remaining(self, motion_id, remaining):
        if self.motion is not None and motion_id == self.motion['id']:
            self.seg_remaining = remaining

    def set_estop(self, pressed):
        """Follow the hardware e-stop: pressed gives estop; released lands on stopped."""
        pressed = bool(pressed)
        if pressed == self.estop:
            return
        self.estop = pressed
        if pressed:
            self.origin = 'manual' if self.state == 'manual' else 'running'
            self._say('E-STOP pressed: motor power cut')
            self._clear_motion()
            self.state = 'estop'
            return
        self.message = 'E-stop released. Motors re-enabled.'
        self.stop_kind = 'estop'
        self._say('E-stop released')
        self._to_stopped()

    def segment_result(self, motion_id, success, error='', message=''):
        """Take the end of a base move from the segment driver."""
        if self.motion is None or motion_id != self.motion['id'] or self.state != 'running':
            return
        label = self.motion['label']
        self._clear_motion()
        if success:
            self._next_seg()
            return
        if error == 'blocked':
            self._fault(f'Path blocked during "{label}". Drive the robot clear in Manual, '
                        'then retry.')
        elif error == 'no_pose':
            self._fault(f'Lost the robot\'s position during "{label}".')
        elif error == 'no_progress':
            self._fault(f'The base stopped making progress during "{label}". '
                        + (message or 'Check the safety state on the Health screen.'))
        else:
            self._fault(f'"{label}" failed: {message or error or "unknown error"}')

    def tick(self, dt):
        """Advance the timers by ``dt`` seconds."""
        dt = max(0.0, dt)
        state = self.state
        if state == 'running':
            seg = self._seg()
            if seg is None:
                return
            if seg['k'] in ('move', 'turn', 'spin'):
                if self.person_close and self.hold_check == 'on':
                    self.clear_t = 0.0
                    self._clear_motion()
                    self.state = 'hold'
                    self._say('Hold: someone is close')
            elif seg['k'] == 'arm':
                seg['t'] += dt
                if seg['t'] >= seg['dur']:
                    if seg['a'] in ('stow', 'wave'):
                        self.arm_stowed = True
                    self._next_seg()
        elif state == 'hold':
            if self.person_close:
                self.clear_t = 0.0
            else:
                self.clear_t += dt
                if self.clear_t >= self.cfg['hold_clear_s']:
                    self._say('Area clear: resuming')
                    self.state = 'running'
                    self._start_seg()
        elif state == 'gate':
            self.gate_t += dt
            if self.mode == '4' and self.gate_t > self.cfg['gate_fallback_s']:
                self._say('No START in time: falling back to the replay')
                self._start_feature('replay')
        elif state == 'live':
            self.live_t += dt
            self.live_idle += dt   # no leader arm: nothing ever resets it
            if self.live_idle > self.cfg['live_idle_s']:
                self._end_teleop(f'Auto-ended after {self.cfg["live_idle_s"]:.0f} s '
                                 'with no leader motion')
        elif state in ('stowing', 'stopping'):
            self.stow_t += dt
            if self.stow_t >= self.cfg['arm_stow_s']:
                self.arm_stowed = True
                self.state = self.stow_next

    # ----- commands -----------------------------------------------------------------------

    def command(self, action, args=()):
        """Carry out a command. Return (accepted, reason)."""
        args = list(args)
        if action not in self.allowed_actions():
            return False, f'"{action}" is not available while {self._state_words()}'
        handler = getattr(self, '_cmd_' + action)
        result = handler(args)
        return (True, '') if result is None else (False, result)

    def allowed_actions(self):
        # The visitor hold setting can change in every state but the e-stop.
        if self.state == 'estop':
            return []
        return self._allowed() + ['hold_check']

    def _allowed(self):
        s = self.state
        if s == 'idle':
            return ['start_mode', 'run_feature']
        if s == 'running':
            return ['stop', 'go_home', 'reset', 'hold_skip']
        if s == 'hold':
            return ['stop', 'go_home', 'reset', 'hold_skip']
        if s == 'gate':
            out = ['stop', 'teleop_skip' if self.mode == '4' else 'teleop_cancel']
            if all(c['ok'] for c in self.gate_checks()):
                out.append('teleop_start')
            return out
        if s == 'live':
            return ['stop', 'teleop_stop']
        if s == 'stowing':
            return ['stop']
        if s == 'ended':
            return ['stop', 'save_run', 'discard_run']
        if s == 'stopped':
            out = ['go_home', 'reset', 'start_mode']
            if self.can_resume():
                out.append('resume')
            return out
        if s == 'manual':
            out = ['stop', 'manual_sub', 'exit_manual', 'manual_checks']
            if self.manual_sub == 'arm':
                out.append('manual_teleop')
            return out
        return []   # stopping

    def can_resume(self):
        return self.state == 'stopped' and (self.feat is not None or self.origin == 'manual')

    def _cmd_start_mode(self, args):
        mode = str(args[0]) if args else ''
        if mode not in ('1', '2', '3', '4', '5'):
            return f'Unknown mode "{mode}"'
        if self.state == 'stopped' and mode != '3':
            return 'After a stop, only Manual can be started; use Resume, Go home or Reset'
        if mode != '3' and self.pose is None:
            return 'The robot does not know where it is yet'
        if mode == '2' and self.location() == 'home':
            return 'Already at home'
        self._start_mode(mode)

    def _cmd_run_feature(self, args):
        name = args[0] if args else ''
        if name not in self.features:
            return f'Unknown feature "{name}"'
        if self.pose is None:
            return 'The robot does not know where it is yet'
        ready = self.needs(name)
        if ready is not True:
            return ready
        loop = len(args) > 1 and args[1] == 'loop' and self.features[name].get('loopable', False)
        self._start_playlist([name], 0, loop, 'single')

    def _cmd_stop(self, args):
        self.origin = self.state
        self.message = 'Stopped by staff'
        self.stop_kind = 'staff'
        self._say('STOP: ' + self.message)
        self._to_stopped()

    def _cmd_resume(self, args):
        if self.origin == 'manual':
            self.state = 'manual'
            self.manual_checks = False
            self._say('Resumed manual (safety checks off)')
            return
        self._say('Resume')
        if not self.segs or self.feat in ('replay', 'greet', 'teleop'):
            # These end where they start, so they start again; so does a feature that never
            # started because the robot was in the wrong place.
            self._start_feature(self.feat)
            return
        self.state = 'running'
        self._start_seg()

    def _cmd_go_home(self, args):
        self._clear_motion()
        self.arm_stowed = True
        if self.location() == 'home':
            self._to_idle()
            return
        self._start_playlist(['go_home'], 0, False, '2')

    def _cmd_reset(self, args):
        self._clear_motion()
        self.arm_stowed = True
        mode = self.mode
        if mode in ('1', '4', '5'):
            self._say('Reset: home, then restart')
            if mode == '5' and self.location() != 'home':
                self._start_playlist(['go_home', 'teleop'], 1, True, '5')
            else:
                self._start_mode(mode)
            return
        self._cmd_go_home(args)

    def _cmd_teleop_start(self, args):
        self.live_t = 0.0
        self.live_idle = 0.0
        self.arm_stowed = False
        self.state = 'live'
        self._say(f'Teleop START · recording demo #{self.demos + 1}')

    def _cmd_teleop_stop(self, args):
        self._end_teleop('Ended with STOP')

    def _cmd_teleop_skip(self, args):
        self._say('Skipped teleop: replay instead')
        self._start_feature('replay')

    def _cmd_teleop_cancel(self, args):
        self._say('Teleop cancelled')
        if self.mode == '3':
            self.feat = None
            self.playlist = []
            self.state = 'manual'
        else:
            self._to_idle()

    def _cmd_save_run(self, args):
        self.demos += 1
        self.last_demo = self.demos
        self._say(f'Saved demo #{self.demos}')
        self._feature_done()

    def _cmd_discard_run(self, args):
        self._say('Discarded the run')
        self._feature_done()

    def _cmd_manual_sub(self, args):
        sub = args[0] if args else ''
        if sub not in ('drive', 'arm'):
            return f'Unknown manual sub-mode "{sub}"'
        self.manual_sub = sub

    def _cmd_manual_teleop(self, args):
        self._start_playlist(['teleop'], 0, False, '3')

    def _cmd_manual_checks(self, args):
        value = args[0] if args else ''
        if value not in ('on', 'off'):
            return f'Unknown setting "{value}": on or off'
        self.manual_checks = value == 'on'
        self._say('Manual safety checks ' + value)

    def _cmd_hold_skip(self, args):
        if self.hold_check == 'on':
            self.hold_check = 'off_loop'
        self._say('False alarm: no holds until this loop ends')
        self._resume_from_hold()

    def _cmd_hold_check(self, args):
        value = args[0] if args else ''
        if value not in ('on', 'off'):
            return f'Unknown setting "{value}": on or off'
        self.hold_check = value
        self._say('Visitor hold ' + ('on' if value == 'on' else 'off until turned back on'))
        if value == 'off':
            self._resume_from_hold()

    def _resume_from_hold(self):
        if self.state == 'hold':
            self.state = 'running'
            self._start_seg()

    def _end_loop_skip(self):
        if self.hold_check == 'off_loop':
            self.hold_check = 'on'
            self._say('Visitor hold back on')

    def _cmd_exit_manual(self, args):
        self._say('Manual ended')
        self._to_idle()

    # ----- queries ------------------------------------------------------------------------

    def location(self):
        """Return the key of the marked spot the robot is on, or 'other'."""
        if self.pose is None:
            return 'other'
        best, best_d = 'other', self.cfg['location_tolerance']
        for name, (x, y) in self.poses.items():
            d = math.hypot(self.pose[0] - x, self.pose[1] - y)
            if d < best_d:
                best, best_d = name, d
        return best

    def facing(self, heading):
        return (self.pose is not None and abs(wrap(self.pose[2] - self.headings[heading]))
                < self.cfg['heading_tolerance'])

    def needs(self, name):
        """Return True if feature ``name`` can start here, else the reason as text."""
        need = self.features[name].get('needs', {})
        here = self.location()
        ok = True
        if 'at' in need and here not in need['at']:
            ok = False
        if 'not_at' in need and here in need['not_at']:
            ok = False
        if 'facing' in need and not self.facing(need['facing']):
            ok = False
        return True if ok else need.get('reason', 'Not available here')

    def manual_drive_allowed(self):
        """Return whether manual drive commands may reach the base now."""
        return (self.state == 'manual' and self.manual_sub == 'drive' and self.arm_stowed
                and not (self.manual_checks and self.person_close))

    def safety_bypass(self):
        """Return whether the safety gate's sensor blocks are off (Manual, checks off)."""
        return self.state == 'manual' and not self.manual_checks

    def gate_checks(self):
        arm = self.arm_checks_pass
        label = self.cfg['location_labels'].get(self.location(), '')
        return [
            {'name': 'Base parked, brakes on', 'ok': not self.base_moving, 'detail': label},
            {'name': 'Arm stowed and holding', 'ok': arm and self.arm_stowed,
             'detail': 'Placeholder' if arm else ARM_DETAIL},
            {'name': 'Leader arm streaming on CAN', 'ok': arm,
             'detail': 'Placeholder' if arm else ARM_DETAIL},
            {'name': 'Leader matches robot pose', 'ok': arm,
             'detail': 'Placeholder' if arm else ARM_DETAIL},
        ]

    def eta(self):
        """Return (seconds left in the playlist, a teleop step is still ahead)."""
        total = 0.0
        teleop = self.feat == 'teleop'
        if self.state in MOVING_STATES:
            for j in range(self.si, len(self.segs)):
                seg = self.segs[j]
                estimate = self._seg_estimate(seg)
                if j == self.si:
                    if seg['k'] == 'arm':
                        estimate = max(0.0, seg['dur'] - seg['t'])
                    elif self.seg_remaining is not None:
                        estimate = self.seg_remaining
                total += estimate
        for name in self.playlist[self.pi + 1:]:
            if name == 'teleop':
                teleop = True
            total += self.features[name].get('nominal_s', 0.0)
        return total, teleop

    def remaining_waypoints(self):
        """Return the (x, y) targets of the base moves still ahead in this feature."""
        if self.state not in MOVING_STATES:
            return []
        return [self.poses[seg['pose']] for seg in self.segs[self.si:] if seg['k'] == 'move']

    def snapshot(self):
        """Return everything the panel and the TV need, as a dict of plain values."""
        seg = self._seg() if self.state in ('running', 'hold', 'stopped', 'stopping') else None
        total, teleop = self.eta()
        here = self.location()
        features = []
        for name in self.cfg['feature_order']:
            ready = self.needs(name) if self.pose is not None else 'No position yet'
            features.append({
                'name': name, 'label': self.features[name]['label'],
                'ready': ready is True, 'reason': '' if ready is True else ready,
                'loopable': bool(self.features[name].get('loopable', False)),
                'arm': bool(self.features[name].get('arm', False))})
        gate = self.state == 'gate'
        return {
            'state': self.state,
            'mode': self.mode or 'none',
            'mode_label': self.modes[self.mode]['label'] if self.mode else 'Idle',
            'playlist': list(self.playlist),
            'playlist_labels': [self.features[f]['label'] for f in self.playlist],
            'index': min(self.pi, max(len(self.playlist) - 1, 0)),
            'feature': self.feat or '',
            'step_label': self._label(seg) if seg else '',
            'step_is_arm': bool(seg and seg['k'] == 'arm'),
            'eta_s': total if self.state in MOVING_STATES else -1.0,
            'eta_plus_teleop': teleop,
            'loop': self.loop,
            'loop_count': self.runs,
            'location': here,
            'location_label': self.cfg['location_labels'].get(here, here),
            'pose': self.pose,
            'base_moving': self.base_moving,
            'arm_stowed': self.arm_stowed,
            'arm_integrated': False,
            'manual_sub': self.manual_sub,
            'manual_blocked': self.state == 'manual' and self.manual_checks and self.person_close,
            'manual_checks': self.manual_checks,
            'hold_check': self.hold_check,
            'hold_clear_s': self.clear_t if self.state == 'hold' else 0.0,
            'hold_clear_needed_s': self.cfg['hold_clear_s'],
            'person_close': self.person_close,
            'checks': self.gate_checks() if gate else [],
            'gate_remaining_s': (max(0.0, self.cfg['gate_fallback_s'] - self.gate_t)
                                 if gate and self.mode == '4' else -1.0),
            'live_elapsed_s': self.live_t,
            'live_idle_s': self.live_idle,
            'live_idle_limit_s': self.cfg['live_idle_s'],
            'ended_reason': self.ended_reason,
            'ended_length_s': self.ended_length,
            'demo_count': self.demos,
            'last_demo': self.last_demo,
            'message': self.message if self.state in ('stopped', 'stopping') else '',
            'stop_kind': self.stop_kind if self.state in ('stopped', 'stopping') else '',
            'can_resume': self.can_resume(),
            'notice': self._notice(seg),
            'features': features,
            'allowed_actions': self.allowed_actions(),
        }

    # ----- internals ----------------------------------------------------------------------

    def _say(self, text):
        self.log.append(text)

    def _state_words(self):
        return {'idle': 'idle', 'estop': 'the e-stop is pressed'}.get(
            self.state, 'the robot is ' + self.state)

    def _seg(self):
        return self.segs[self.si] if 0 <= self.si < len(self.segs) else None

    def _label(self, seg):
        return seg['label'].replace('{last_demo}', str(self.last_demo))

    def _notice(self, seg):
        if self.state == 'running' and seg and seg['k'] == 'arm':
            return ARM_NOTICE.format(label=self._label(seg))
        if self.state in ('gate', 'live', 'stowing', 'ended'):
            return ('Teleop needs the arm, which is not integrated yet: this screen is a '
                    'placeholder and nothing moves.')
        if self.state == 'manual' and self.manual_sub == 'arm':
            return 'Arm teleop needs the arm, which is not integrated yet.'
        return ''

    def _clear_motion(self):
        self.motion = None
        self.seg_remaining = None

    def _request_motion(self, **goal):
        self._motion_id += 1
        goal['id'] = self._motion_id
        self.motion = goal
        self.seg_remaining = None

    def _to_idle(self):
        self._end_loop_skip()
        self._clear_motion()
        self.mode = None
        self.feat = None
        self.playlist = []
        self.segs = []
        self.loop = False
        self.state = 'idle'

    def _to_stopped(self):
        self._end_loop_skip()
        self._clear_motion()
        if self.arm_stowed:
            self.state = 'stopped'
        else:
            self.stow_t = 0.0
            self.stow_next = 'stopped'
            self.state = 'stopping'

    def _fault(self, message):
        self.origin = 'running'
        self.message = message
        self.stop_kind = 'fault'
        self._say('Fault: ' + message)
        self._to_stopped()

    def _start_mode(self, mode):
        if mode == '3':
            self.mode = '3'
            self.feat = None
            self.playlist = []
            self.manual_sub = 'drive'
            self.manual_checks = False
            self._say('Start ' + self.modes['3']['label'] + ' (safety checks off)')
            self.state = 'manual'
            return
        if mode == '2':
            self._start_playlist(['go_home'], 0, False, '2')
            return
        playlist = list(self.modes[mode]['playlist'])
        loop_start = 0
        if mode != '5' and self.location() != 'home':
            playlist = ['go_home'] + playlist
            loop_start = 1
        self._start_playlist(playlist, loop_start, True, mode)

    def _start_playlist(self, playlist, loop_start, loop, mode):
        self._end_loop_skip()
        self.playlist = list(playlist)
        self.loop_start = loop_start
        self.loop = loop
        self.mode = mode
        self.pi = 0
        self.runs = 1
        self._say('Start ' + self.modes[mode]['label'])
        self._start_feature(self.playlist[0])

    def _start_feature(self, name):
        ready = self.needs(name)
        self.feat = name
        if ready is not True:
            self.segs = []
            self.si = 0
            self._fault(f'{self.features[name]["label"]}: {ready}')
            return
        self.segs = self._compile(name)
        self.si = 0
        self._say('Feature: ' + self.features[name]['label'])
        self.state = 'running'
        self._start_seg()

    def _feature_done(self):
        self.pi += 1
        if self.pi >= len(self.playlist):
            if not self.loop:
                self._finish()
                return
            self.pi = self.loop_start
            self._end_loop_skip()
            self.runs += 1
            self._say(f'Loop {self.runs} starts')
        self._start_feature(self.playlist[self.pi])

    def _finish(self):
        if self.mode == '3':
            self.feat = None
            self.playlist = []
            self.segs = []
            self.state = 'manual'
            self._say('Back to manual control')
            return
        self._say(self.modes[self.mode]['label'] + ' finished')
        self._to_idle()

    def _compile(self, name):
        """Return the feature's steps as segment dicts."""
        feature = self.features[name]
        segs = []
        if 'route' in feature:
            return self._compile_go_home(feature)
        for step in feature['steps']:
            if 'move' in step:
                segs.append({'k': 'move', 'pose': step['move'], 'h': step.get('heading'),
                             'slow': bool(step.get('slow', False)), 'label': step['label']})
            elif 'turn' in step:
                segs.append({'k': 'turn', 'h': step['turn'], 'label': step['label']})
            elif 'spin' in step:
                segs.append({'k': 'spin', 'd': float(step['spin']), 'end': None,
                             'label': step['label']})
            elif 'arm' in step:
                segs.append({'k': 'arm', 'a': step['arm'], 'dur': float(step['duration']),
                             't': 0.0, 'label': step['label']})
            elif 'teleop' in step:
                segs.append({'k': 'teleop', 'label': step['label']})
        return segs

    def _compile_go_home(self, feature):
        """Return the steps home from where the robot is (see config/booth_demo.yaml)."""
        here = self.location()
        heading = feature['home_heading']
        then = feature['then']
        segs = []
        if here == 'other' and self.pose is not None:
            # Off the marked spots (stopped mid-move): every prewritten move is a straight
            # line between two spots, so first go straight back to the nearest one.
            here = min(self.poses, key=lambda k: math.hypot(self.pose[0] - self.poses[k][0],
                                                            self.pose[1] - self.poses[k][1]))
            segs.append({'k': 'move', 'pose': here, 'h': None, 'slow': True,
                         'label': 'Back to ' + self.cfg['location_labels'][here].lower()})
            if here == 'home':
                return segs + ([] if self.facing(heading) else
                               [{'k': 'turn', 'h': heading, 'label': 'Turn toward home'}])
        if self.facing(heading) and (here in then or here == feature['turn_at']):
            rest = then[then.index(here) + 1:] if here in then else then
        else:
            route = feature['route'].get(here, feature['route']['other'])
            for index, spot in enumerate(route):
                segs.append({'k': 'move', 'pose': spot, 'h': None,
                             'slow': here == 'dock' and index == 0,
                             'label': feature['route_labels'].get(spot, 'Glide')})
            segs.append({'k': 'turn', 'h': heading, 'label': 'Turn toward home'})
            rest = then
        for spot in rest:
            segs.append({'k': 'move', 'pose': spot, 'h': heading, 'slow': False,
                         'label': feature['then_labels'].get(spot, 'Glide')})
        return segs

    def _seg_estimate(self, seg):
        """Return a rough length of a step that has not started, in seconds."""
        kind = seg['k']
        if kind == 'arm':
            return seg['dur']
        if kind == 'move':
            return 0.6 / (self.speeds['linear_slow'] if seg['slow']
                          else self.speeds['linear']) + 1.0
        if kind == 'turn':
            return 1.6 / self.speeds['angular'] + 1.0
        if kind == 'spin':
            return abs(seg['d']) / self.speeds['angular'] + 1.5
        return 0.0

    def _next_seg(self):
        self.si += 1
        self._start_seg()

    def _start_seg(self):
        seg = self._seg()
        if seg is None:
            self._feature_done()
            return
        kind = seg['k']
        if kind == 'move':
            x, y = self.poses[seg['pose']]
            self._request_motion(
                kind='move', x=x, y=y,
                yaw=self.headings[seg['h']] if seg['h'] else 0.0,
                keep_heading=seg['h'] is None, delta_yaw=0.0, slow=seg['slow'],
                label=seg['label'])
        elif kind == 'turn':
            if self.pose is not None and abs(
                    wrap(self.headings[seg['h']] - self.pose[2])) < 0.03:
                self._next_seg()
                return
            self._request_motion(kind='turn', x=0.0, y=0.0, yaw=self.headings[seg['h']],
                                 keep_heading=False, delta_yaw=0.0, slow=False,
                                 label=seg['label'])
        elif kind == 'spin':
            if seg['end'] is None and self.pose is not None:
                # First start: remember where the spin ends, so that a spin resumed after a
                # hold or a stop finishes at the same heading by the shorter way.
                seg['end'] = wrap(self.pose[2] + seg['d'])
                self._request_motion(kind='spin', x=0.0, y=0.0, yaw=0.0, keep_heading=False,
                                     delta_yaw=seg['d'], slow=False, label=seg['label'])
            else:
                self._request_motion(kind='turn', x=0.0, y=0.0, yaw=seg['end'] or 0.0,
                                     keep_heading=False, delta_yaw=0.0, slow=False,
                                     label=seg['label'])
        elif kind == 'arm':
            seg['t'] = 0.0
            self.arm_stowed = False
        elif kind == 'teleop':
            self.gate_t = 0.0
            self.state = 'gate'
            self._say('Teleop gate: waiting for START')

    def _end_teleop(self, reason):
        self.ended_reason = reason
        self.ended_length = self.live_t
        self._say('Teleop ended: ' + reason)
        self.stow_t = 0.0
        self.stow_next = 'ended'
        self.state = 'stowing'
