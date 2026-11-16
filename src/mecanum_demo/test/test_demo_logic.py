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

"""The supervisor's state machine against the flows of docs/Booth demo simulator.html."""

from demo_harness import Harness
import pytest


def test_starts_idle_at_home_with_the_modes_offered():
    h = Harness()
    snap = h.logic.snapshot()
    assert snap['state'] == 'idle'
    assert snap['location'] == 'home'
    assert snap['allowed_actions'] == ['start_mode', 'run_feature', 'hold_check']
    ready = {f['name']: f['ready'] for f in snap['features']}
    assert ready == {'leave_home': True, 'spin': False, 'dock': False, 'replay': False,
                     'teleop': True, 'back_out': False, 'greet': False, 'go_home': False}


def test_mode_1_plays_the_show_loop_and_restarts():
    h = Harness()
    h.ok('start_mode', '1')
    assert h.logic.snapshot()['playlist'] == [
        'leave_home', 'spin', 'dock', 'replay', 'back_out', 'greet', 'go_home']
    h.run_until_feature('dock')
    assert h.at('stage') and h.logic.facing('E')
    h.run_until_feature('replay')
    assert h.at('dock')
    h.run(lambda logic: logic.runs == 2)
    assert h.at('home') and h.logic.facing('W')
    assert h.logic.feat == 'leave_home' and h.logic.state == 'running'
    assert h.moves[:4] == ['Sidestep out of home', 'Glide across the booth',
                           'Sidestep to centre stage', 'Spin flourish']


def test_arm_steps_are_timed_placeholders_with_a_notice():
    h = Harness(at='dock', heading='E')
    h.ok('run_feature', 'replay')
    snap = h.logic.snapshot()
    assert snap['state'] == 'running' and snap['step_is_arm']
    assert 'not integrated' in snap['notice']
    assert h.logic.motion is None and not snap['arm_stowed']
    h.logic.tick(1.5)
    assert h.logic.si == 0
    h.logic.tick(0.2)
    assert h.logic.si == 1
    assert h.logic.snapshot()['step_label'] == 'Replaying recorded demo #37'
    h.run_until_state('idle')
    assert h.logic.arm_stowed and h.moves == []


def test_mode_2_goes_home_and_is_refused_at_home():
    h = Harness()
    accepted, reason = h.cmd('start_mode', '2')
    assert not accepted and reason == 'Already at home'
    h = Harness(at='dock', heading='E')
    h.ok('start_mode', '2')
    h.run_until_state('idle')
    assert h.at('home') and h.logic.facing('W')
    assert h.moves == ['Back out', 'Slide out', 'Glide clear', 'Sidestep to centre stage',
                       'Turn toward home', 'Sidestep clear', 'Glide toward home',
                       'Sidestep into home']
    h = Harness(at='w1', heading='W')
    h.ok('start_mode', '2')
    h.run_until_state('idle')
    assert h.moves == ['Sidestep into home']


def test_a_mode_started_away_from_home_goes_home_first_and_loops_without_it():
    h = Harness(at='stage', heading='E')
    h.ok('start_mode', '1')
    assert h.logic.playlist[0] == 'go_home' and h.logic.loop_start == 1
    h.run(lambda logic: logic.runs == 2)
    assert h.logic.feat == 'leave_home'


def test_single_feature_once_and_loop():
    h = Harness(at='stage', heading='W')
    accepted, reason = h.cmd('run_feature', 'dock')
    assert not accepted and reason == 'Needs centre stage, facing the toilet'
    h.ok('run_feature', 'spin')
    h.run_until_state('idle')
    assert h.logic.facing('E')
    h.ok('run_feature', 'dock', 'loop')      # not loopable: runs once
    h.run_until_state('idle')
    assert h.at('dock')
    h = Harness(at='stage', heading='W')
    h.ok('run_feature', 'spin', 'loop')
    h.run(lambda logic: logic.runs == 3)
    assert h.logic.state == 'running'


def test_hold_cancels_the_move_and_resumes_three_seconds_after_the_area_clears():
    h = Harness()
    h.ok('start_mode', '1')
    first = h.logic.motion['id']
    h.logic.set_person_close(True)
    h.logic.tick(0.1)
    assert h.logic.state == 'hold' and h.logic.motion is None
    h.logic.tick(5.0)
    assert h.logic.state == 'hold'
    h.logic.set_person_close(False)
    h.logic.tick(2.0)
    h.logic.set_person_close(True)      # back again: the wait starts over
    h.logic.tick(0.1)
    h.logic.set_person_close(False)
    h.logic.tick(2.9)
    assert h.logic.state == 'hold'
    h.logic.tick(0.2)
    assert h.logic.state == 'running'
    assert h.logic.motion['label'] == 'Sidestep out of home' and h.logic.motion['id'] != first


def test_a_spin_resumed_after_a_hold_ends_at_the_same_heading():
    h = Harness(at='stage', heading='W')
    h.ok('run_feature', 'spin')
    assert h.logic.motion['kind'] == 'spin'
    x, y, yaw = h.logic.pose
    h.logic.set_pose((x, y, yaw - 2.0))      # part of the way round
    h.logic.set_person_close(True)
    h.logic.tick(0.1)
    h.logic.set_person_close(False)
    h.logic.tick(3.1)
    assert h.logic.motion['kind'] == 'turn'
    assert h.logic.motion['yaw'] == pytest.approx(0.0, abs=1e-6)


def test_stop_then_resume_continues_the_same_step():
    h = Harness()
    h.ok('start_mode', '1')
    h.finish_motion()
    h.ok('stop')
    snap = h.logic.snapshot()
    assert snap['state'] == 'stopped' and snap['stop_kind'] == 'staff'
    assert snap['message'] == 'Stopped by staff' and h.logic.motion is None
    assert set(snap['allowed_actions']) == {
        'go_home', 'reset', 'start_mode', 'resume', 'hold_check'}
    h.ok('resume')
    assert h.logic.state == 'running'
    assert h.logic.motion['label'] == 'Glide across the booth'


def test_stop_with_the_arm_out_stows_first():
    h = Harness(at='dock', heading='E')
    h.ok('run_feature', 'replay')
    h.ok('stop')
    assert h.logic.state == 'stopping' and h.logic.allowed_actions() == ['hold_check']
    h.logic.tick(1.5)
    assert h.logic.state == 'stopped' and h.logic.arm_stowed
    h.ok('resume')                           # the replay starts again
    assert h.logic.si == 0 and h.logic.state == 'running'


def test_stop_is_refused_when_nothing_runs():
    h = Harness()
    accepted, reason = h.cmd('stop')
    assert not accepted and 'idle' in reason


def test_a_blocked_path_is_a_fault_and_manual_is_offered():
    h = Harness()
    h.ok('start_mode', '1')
    h.finish_motion(success=False, error='blocked')
    snap = h.logic.snapshot()
    assert snap['state'] == 'stopped' and snap['stop_kind'] == 'fault'
    assert snap['message'].startswith('Path blocked during "Sidestep out of home"')
    accepted, _ = h.cmd('start_mode', '1')
    assert not accepted
    h.ok('start_mode', '3')
    assert h.logic.state == 'manual'


def test_reset_goes_home_and_restarts_the_mode():
    h = Harness()
    h.ok('start_mode', '4')
    h.run_until_feature('dock')
    h.ok('stop')
    h.ok('reset')
    assert h.logic.mode == '4' and h.logic.playlist[0] == 'go_home'
    h.run_until_feature('leave_home')
    assert h.at('home')


def test_go_home_from_stopped_ends_in_idle():
    h = Harness()
    h.ok('start_mode', '1')
    h.run_until_feature('spin')
    h.ok('stop')
    h.ok('go_home')
    assert h.logic.mode == '2'
    h.run_until_state('idle')
    assert h.at('home')


def test_a_feature_in_the_wrong_place_faults_with_the_reason():
    h = Harness()
    h.ok('start_mode', '1')
    h.finish_motion()
    h.ok('stop')
    h.ok('go_home')        # home again, but now ask the playlist to continue from "spin"
    h.run_until_state('idle')
    h.logic._start_playlist(['spin'], 0, False, 'single')
    snap = h.logic.snapshot()
    assert snap['state'] == 'stopped' and snap['message'] == 'Spin: Needs to be at centre stage'


def test_mode_4_gate_falls_back_to_the_replay_after_20_s():
    h = Harness()
    h.ok('start_mode', '4')
    h.run_until_state('gate')
    snap = h.logic.snapshot()
    assert [c['ok'] for c in snap['checks']] == [True, False, False, False]
    assert 'teleop_start' not in snap['allowed_actions']
    assert 'teleop_skip' in snap['allowed_actions']
    assert snap['gate_remaining_s'] == pytest.approx(20.0)
    assert 'not integrated' in snap['notice']
    h.logic.tick(19.0)
    assert h.logic.state == 'gate'
    h.logic.tick(1.5)
    assert h.logic.state == 'running' and h.logic.feat == 'replay'
    h.run_until_feature('back_out')         # the playlist goes on after the replay
    assert h.logic.pi == 4


def test_teleop_session_with_the_development_switch():
    h = Harness()
    h.logic.set_arm_checks_pass(True)
    h.ok('start_mode', '5')
    assert h.logic.state == 'gate'
    snap = h.logic.snapshot()
    assert snap['gate_remaining_s'] == -1.0
    assert 'teleop_cancel' in snap['allowed_actions']
    h.logic.tick(60.0)                      # Mode 5 waits as long as needed
    assert h.logic.state == 'gate'
    h.ok('teleop_start')
    assert h.logic.state == 'live' and not h.logic.arm_stowed
    h.logic.tick(4.0)
    h.ok('teleop_stop')
    assert h.logic.state == 'stowing'
    h.logic.tick(1.5)
    snap = h.logic.snapshot()
    assert snap['state'] == 'ended' and snap['ended_reason'] == 'Ended with STOP'
    assert snap['ended_length_s'] == pytest.approx(4.0)
    h.ok('save_run')
    assert h.logic.demos == 38 and h.logic.last_demo == 38
    assert h.logic.state == 'gate' and h.logic.runs == 2     # waits for the next visitor


def test_live_session_ends_itself_after_15_s_idle_and_discard_keeps_the_count():
    h = Harness()
    h.logic.set_arm_checks_pass(True)
    h.ok('run_feature', 'teleop')
    h.ok('teleop_start')
    h.logic.tick(14.0)
    assert h.logic.state == 'live'
    h.logic.tick(1.5)
    assert h.logic.state == 'stowing'
    h.logic.tick(1.5)
    assert h.logic.snapshot()['ended_reason'].startswith('Auto-ended after 15 s')
    h.ok('discard_run')
    assert h.logic.state == 'idle' and h.logic.demos == 37


def test_teleop_start_needs_every_check():
    h = Harness()
    h.ok('start_mode', '5')
    accepted, _ = h.cmd('teleop_start')
    assert not accepted
    h.logic.set_arm_checks_pass(True)
    h.logic.set_base_moving(True)
    accepted, _ = h.cmd('teleop_start')
    assert not accepted
    h.logic.set_base_moving(False)
    h.ok('teleop_start')
    h.ok('stop')
    assert h.logic.state == 'stopping'


def test_teleop_cancel_ends_mode_5():
    h = Harness()
    h.ok('start_mode', '5')
    h.ok('teleop_cancel')
    assert h.logic.state == 'idle' and h.logic.mode is None


def test_manual_drive_gating_and_arm_sub_mode():
    h = Harness()
    assert not h.logic.manual_drive_allowed()
    h.ok('start_mode', '3')
    assert h.logic.manual_drive_allowed()
    h.ok('manual_checks', 'on')                 # with the checks on, someone close blocks
    h.logic.set_person_close(True)
    assert not h.logic.manual_drive_allowed()
    assert h.logic.snapshot()['manual_blocked']
    h.logic.set_person_close(False)
    accepted, _ = h.cmd('manual_teleop')
    assert not accepted
    h.ok('manual_sub', 'arm')
    assert not h.logic.manual_drive_allowed()
    h.ok('manual_teleop')
    assert h.logic.state == 'gate' and h.logic.mode == '3'
    h.ok('teleop_cancel')
    assert h.logic.state == 'manual'
    h.ok('manual_sub', 'drive')
    h.ok('stop')
    assert h.logic.state == 'stopped' and h.logic.can_resume()
    h.ok('resume')
    assert h.logic.state == 'manual'
    h.ok('exit_manual')
    assert h.logic.state == 'idle'


def test_estop_from_running_lands_on_stopped_and_can_resume():
    h = Harness()
    h.ok('start_mode', '1')
    h.logic.set_estop(True)
    assert h.logic.state == 'estop' and h.logic.motion is None
    assert h.logic.allowed_actions() == []
    h.logic.tick(10.0)
    assert h.logic.state == 'estop'
    h.logic.set_estop(False)
    snap = h.logic.snapshot()
    assert snap['state'] == 'stopped' and snap['stop_kind'] == 'estop'
    assert snap['can_resume']
    h.ok('resume')
    assert h.logic.motion['label'] == 'Sidestep out of home'


def test_estop_from_idle_offers_no_resume():
    h = Harness()
    h.logic.set_estop(True)
    h.logic.set_estop(False)
    assert h.logic.state == 'stopped' and not h.logic.can_resume()
    h.ok('go_home')
    assert h.logic.state == 'idle'


def test_stale_and_unexpected_segment_results_are_ignored():
    h = Harness()
    h.ok('start_mode', '1')
    old = h.logic.motion['id']
    h.ok('stop')
    h.logic.segment_result(old, True)
    assert h.logic.state == 'stopped'
    h.ok('resume')
    h.logic.segment_result(old, False, 'blocked')
    assert h.logic.state == 'running'


def test_no_position_refuses_to_start():
    h = Harness()
    h.logic.set_pose(None)
    accepted, reason = h.cmd('start_mode', '1')
    assert not accepted and 'where it is' in reason
    h.ok('start_mode', '3')


def test_eta_counts_down_through_the_playlist():
    h = Harness()
    h.ok('start_mode', '4')
    total, teleop = h.logic.eta()
    assert teleop and total > 40.0
    h.logic.set_segment_remaining(h.logic.motion['id'], 1.0)
    assert h.logic.eta()[0] < total
    h.run_until_feature('back_out')
    assert not h.logic.eta()[1]


def test_go_home_from_off_the_spots_goes_back_to_the_nearest_spot_first():
    h = Harness()
    h.ok('start_mode', '1')
    h.finish_motion()                                  # at w1
    h.logic.set_pose((0.0, -0.2, h.cfg['headings']['W']))   # stopped halfway to b1
    h.ok('stop')
    h.ok('go_home')
    assert h.logic.motion['label'] == 'Back to near the dock'      # a1 is nearest
    h.run_until_state('idle')
    assert h.at('home') and h.moves[-1] == 'Sidestep into home'


def test_false_alarm_resumes_at_once_and_skips_holds_until_the_loop_ends():
    h = Harness()
    h.ok('start_mode', '1')
    h.logic.set_person_close(True)
    h.logic.tick(0.1)
    assert h.logic.state == 'hold'
    h.ok('hold_skip')
    snap = h.logic.snapshot()
    assert snap['state'] == 'running' and snap['hold_check'] == 'off_loop'
    assert h.logic.motion is not None          # the move starts again at once
    # Someone stays close for the rest of the loop: every move is ticked, and none holds.
    for _ in range(400):
        if h.logic.runs == 2:
            break
        h.logic.tick(0.5)
        assert h.logic.state != 'hold'
        if h.logic.motion is not None:
            h.finish_motion()
    assert h.logic.hold_check == 'on'
    # The next loop holds again.
    h.logic.tick(0.1)
    assert h.logic.state == 'hold'


def test_a_stop_ends_the_false_alarm_skip():
    h = Harness()
    h.ok('start_mode', '1')
    h.ok('hold_skip')
    assert h.logic.hold_check == 'off_loop'
    h.ok('stop')
    assert h.logic.hold_check == 'on'


def test_the_hold_setting_stays_off_until_turned_back_on():
    h = Harness()
    h.ok('hold_check', 'off')
    h.ok('start_mode', '1')
    h.logic.set_person_close(True)
    for _ in range(800):                            # two whole loops, nobody held for
        if h.logic.runs == 3:
            break
        h.logic.tick(0.5)
        assert h.logic.state != 'hold'
        if h.logic.motion is not None:
            h.finish_motion()
    h.ok('stop')
    h.ok('go_home')
    assert h.logic.hold_check == 'off'
    h.ok('hold_check', 'on')
    h.run_until_state('idle')
    h.ok('start_mode', '1')
    h.logic.tick(0.1)
    assert h.logic.state == 'hold'


def test_turning_the_hold_off_during_a_hold_resumes():
    h = Harness()
    h.ok('start_mode', '1')
    h.logic.set_person_close(True)
    h.logic.tick(0.1)
    h.ok('hold_check', 'off')
    assert h.logic.state == 'running' and h.logic.motion is not None


def test_the_hold_setting_cannot_change_during_an_estop():
    h = Harness()
    h.logic.set_estop(True)
    accepted, _ = h.cmd('hold_check', 'off')
    assert not accepted


def test_manual_starts_with_the_checks_off_every_time():
    h = Harness()
    h.ok('start_mode', '3')
    snap = h.logic.snapshot()
    assert not snap['manual_checks'] and h.logic.safety_bypass()
    h.logic.set_person_close(True)
    assert h.logic.manual_drive_allowed()          # the operator's judgement
    assert not snap['manual_blocked']
    h.ok('manual_checks', 'on')
    assert not h.logic.manual_drive_allowed() and not h.logic.safety_bypass()
    assert h.logic.snapshot()['manual_blocked']
    h.ok('exit_manual')
    assert not h.logic.safety_bypass()
    h.ok('start_mode', '3')
    assert not h.logic.manual_checks


def test_resuming_manual_after_a_stop_turns_the_checks_off_again():
    h = Harness()
    h.ok('start_mode', '3')
    h.ok('manual_checks', 'on')
    h.ok('stop')
    assert not h.logic.safety_bypass()
    h.ok('resume')
    assert h.logic.state == 'manual' and not h.logic.manual_checks
