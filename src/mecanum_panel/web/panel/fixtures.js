// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Sample supervisor states, for looking at each screen without a robot:
//   /panel/?fixture=running   (home, running, hold, hold_off, gate, live, ended, stopped,
//                              manual, manual_checks_on, manual_arm, estop, health, linklost)
// Development only: with a fixture the panel never opens the websocket.

const FEATURES = [
  ['leave_home', 'Leave home', true, '', false, false],
  ['spin', 'Spin', false, 'Needs to be at centre stage', true, false],
  ['dock', 'Dock', false, 'Needs centre stage, facing the toilet', false, false],
  ['replay', 'Replay', false, 'Needs to be docked', true, true],
  ['teleop', 'Teleop', true, '', true, true],
  ['back_out', 'Back out', false, 'Needs to be docked', false, false],
  ['greet', 'Greet', false, 'Needs to be at centre stage', true, false],
  ['go_home', 'Go home', false, 'Already at home', false, false],
].map(([name, label, ready, reason, loopable, arm]) => ({name, label, ready, reason, loopable, arm}));

const PLAYLIST = ['leave_home', 'spin', 'dock', 'replay', 'back_out', 'greet', 'go_home'];
const LABELS = ['Leave home', 'Spin', 'Dock', 'Replay', 'Back out', 'Greet', 'Go home'];

const BASE = {
  state: 'idle', mode: 'none', mode_label: 'Idle', playlist: [], playlist_labels: [], index: 0,
  feature: '', step_label: '', step_is_arm: false, eta_s: -1, eta_plus_teleop: false,
  loop: false, loop_count: 1, location: 'home', location_label: 'At home',
  pose: {x: -0.5, y: 0.35, theta: 3.14159}, pose_valid: true, base_moving: false,
  arm_stowed: true, arm_integrated: false, manual_sub: 'drive', manual_blocked: false,
  manual_checks: false, hold_check: 'on',
  hold_clear_s: 0, hold_clear_needed_s: 3, person_close: false, checks: [],
  gate_remaining_s: -1, live_elapsed_s: 0, live_idle_s: 0, live_idle_limit_s: 15,
  ended_reason: '', ended_length_s: 0, demo_count: 37, last_demo: 37, message: '',
  stop_kind: '', can_resume: false, notice: '', features: FEATURES,
  allowed_actions: ['start_mode', 'run_feature', 'hold_check'],
};

const RUN = {
  mode: '1', mode_label: 'Mode 1 · Show loop', playlist: PLAYLIST, playlist_labels: LABELS,
  loop: true, location: 'stage', location_label: 'At centre stage',
};

const CHECKS = [
  {name: 'Base parked, brakes on', ok: true, detail: 'At centre stage'},
  {name: 'Arm stowed and holding', ok: false, detail: 'Arm not integrated'},
  {name: 'Leader arm streaming on CAN', ok: false, detail: 'Arm not integrated'},
  {name: 'Leader matches robot pose', ok: false, detail: 'Arm not integrated'},
];
const TELEOP_NOTE = 'Teleop needs the arm, which is not integrated yet: this screen is a ' +
  'placeholder and nothing moves.';

export const FIXTURES = {
  home: {},
  running: {...RUN, state: 'running', index: 1, feature: 'spin', step_label: 'Spin flourish',
            eta_s: 61, base_moving: true, allowed_actions: ['stop', 'go_home', 'reset']},
  replay: {...RUN, state: 'running', index: 3, feature: 'replay', location: 'dock',
           location_label: 'Docked', step_label: 'Replaying recorded demo #37', step_is_arm: true,
           eta_s: 38, arm_stowed: false, allowed_actions: ['stop', 'go_home', 'reset'],
           notice: 'Arm action "Replaying recorded demo #37": the arm is not integrated yet, nothing moves.'},
  hold: {...RUN, state: 'hold', index: 1, feature: 'spin', step_label: 'Spin flourish', eta_s: 58,
         person_close: true, allowed_actions: ['stop', 'go_home', 'reset', 'hold_skip', 'hold_check']},
  hold_off: {...RUN, state: 'running', index: 2, feature: 'dock', step_label: 'Glide toward the toilet',
             eta_s: 49, base_moving: true, hold_check: 'off_loop', person_close: true,
             allowed_actions: ['stop', 'go_home', 'reset', 'hold_skip', 'hold_check']},
  gate: {...RUN, mode: '4', mode_label: 'Mode 4 · Show loop + teleop', state: 'gate', index: 3,
         feature: 'teleop', location: 'dock', location_label: 'Docked', checks: CHECKS,
         gate_remaining_s: 14.2, notice: TELEOP_NOTE, allowed_actions: ['stop', 'teleop_skip']},
  live: {...RUN, mode: '5', mode_label: 'Mode 5 · Teleop in place', state: 'live',
         playlist: ['teleop'], playlist_labels: ['Teleop'], feature: 'teleop', arm_stowed: false,
         live_elapsed_s: 42, live_idle_s: 9.5, notice: TELEOP_NOTE,
         allowed_actions: ['stop', 'teleop_stop']},
  ended: {...RUN, mode: '5', mode_label: 'Mode 5 · Teleop in place', state: 'ended',
          playlist: ['teleop'], playlist_labels: ['Teleop'], feature: 'teleop',
          ended_reason: 'Auto-ended after 15 s with no leader motion', ended_length_s: 57,
          notice: TELEOP_NOTE, allowed_actions: ['stop', 'save_run', 'discard_run']},
  stopped: {...RUN, state: 'stopped', index: 2, feature: 'dock', stop_kind: 'fault',
            location: 'other', location_label: 'Off the marked spots',
            message: 'Path blocked during "Slide in sideways". Drive the robot clear in Manual, then retry.',
            can_resume: true, allowed_actions: ['go_home', 'reset', 'start_mode', 'resume']},
  manual: {mode: '3', mode_label: 'Mode 3 · Manual', state: 'manual',
           allowed_actions: ['stop', 'manual_sub', 'exit_manual', 'manual_checks', 'hold_check']},
  manual_checks_on: {mode: '3', mode_label: 'Mode 3 · Manual', state: 'manual', manual_checks: true,
                     person_close: true, manual_blocked: true,
                     allowed_actions: ['stop', 'manual_sub', 'exit_manual', 'manual_checks',
                                       'hold_check']},
  manual_arm: {mode: '3', mode_label: 'Mode 3 · Manual', state: 'manual', manual_sub: 'arm',
               notice: 'Arm teleop needs the arm, which is not integrated yet.',
               allowed_actions: ['stop', 'manual_sub', 'exit_manual', 'manual_teleop',
                                 'manual_checks', 'hold_check']},
  estop: {...RUN, state: 'estop', index: 1, feature: 'spin', allowed_actions: []},
  health: {},
  linklost: null,
};

export const HEALTH = {
  status: [
    {level: 1, name: 'Battery', message: 'Not measured'},
    {level: 0, name: '2D lidar', message: '10.0 Hz'},
    {level: 0, name: 'L2 lidar', message: '5.3 Hz'},
    {level: 0, name: 'Drive', message: '50.0 Hz'},
    {level: 0, name: 'Localization', message: '±2 cm, ±1°'},
    {level: 0, name: 'Safety gate', message: 'Clear'},
    {level: 1, name: 'Arm CAN links', message: 'Arm not integrated'},
  ],
};

export function fixtureState(name) {
  const f = FIXTURES[name];
  return f === null || f === undefined ? null : {...BASE, ...f};
}

// A stand-in for the ROS adapter that never connects anywhere.
export class FixtureRos {
  constructor() {
    this.connected = true;
  }

  onConnection(cb) {
    cb(true);
  }

  subscribe() {}

  advertise() {}

  publish() {
    return true;
  }

  callService(service, type, args) {
    return Promise.resolve({accepted: false, reason: `Fixture: "${args.action}" not sent`});
  }
}
