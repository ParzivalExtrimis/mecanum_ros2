// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Manual: the base on the D-pad (streamed by the drive streamer while a key is held), or the
// arm sub-mode, which opens the teleop gate (an arm placeholder for now). The safety checks
// (visitor hold, cliff sensors, bumpers) start off on every entry: Manual is on the operator's
// judgement, and an external controller on /manual/cmd_vel drives it too.

import {cmdButton, esc, notice, setText, settingSeg} from './common.js';

const KEYS = [['ccw', 'Rotate left'], ['f', 'Forward'], ['cw', 'Rotate right'],
              ['l', 'Strafe left'], ['b', 'Back'], ['r', 'Strafe right']];

export const manual = {
  key(s) {
    return ['manual', s.manual_sub, Boolean(s.notice), s.manual_checks].join('|');
  },

  html(s) {
    const drive = s.manual_sub === 'drive';
    const toggle = '<div class="toggle">' +
      cmdButton(drive ? 'on' : '', 'manual_sub', 'Drive base', '', {args: ['drive']}) +
      cmdButton(drive ? '' : 'on', 'manual_sub', 'Teleop arm', '', {args: ['arm']}) + '</div>';
    const body = drive
      ? '<div class="dpad">' + KEYS.map(([k, label]) =>
        `<button data-dir="${k}">${esc(label)}</button>`).join('') + '</div>'
      : `${notice(s)}` + cmdButton('secondary', 'manual_teleop', 'Open the teleop gate',
                                    'Base stays braked during teleop');
    return {
      content: `<div class="main">${toggle}${body}</div>` +
        '<div class="side"><h2>Manual</h2>' +
        `<span class="muted small">${drive
          ? 'Hold a key to move; the base stops when you let go. Keyboard: W A S D, Q E. ' +
            'A controller on /manual/cmd_vel drives too.'
          : 'The arm is driven from the leader arm at the counter.'}</span>` +
        '<div class="row" style="align-items:center"><span>Safety checks</span>' +
        settingSeg('manual_checks', s.manual_checks) + '</div>' +
        `<span class="small ${s.manual_checks ? 'muted' : 'amber-text'}">${s.manual_checks
          ? 'On: someone close, a cliff or a bumper stops the base.'
          : 'Off: visitors, cliff sensors and bumpers do not stop the base. The e-stop always works.'}` +
        '</span><span class="amber-text" id="m-block"></span></div>',
      actions: cmdButton('primary', 'exit_manual', 'Exit Manual', 'Back to the modes'),
    };
  },

  update(root, s) {
    setText(root, '#m-block', s.manual_blocked ? 'Holding: someone is too close.' : '');
  },
};
