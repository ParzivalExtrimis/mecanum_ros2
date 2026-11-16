// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Running: the playlist, the current step and the time left. Go home ends the mode. In a hold
// (the hold overlay covers the top), the primary action is the false-alarm bypass, which
// resumes at once and skips holds until this loop ends; Go home moves to the side panel.

import {cmdButton, esc, fmtTime, milestones, notice, setText} from './common.js';

export const running = {
  key(s) {
    return ['running', s.playlist.join(), s.index, s.state, Boolean(s.notice)].join('|');
  },

  html(s) {
    const hold = s.state === 'hold';
    return {
      content: '<div class="main"><span class="muted" id="r-mode"></span>' +
        '<div><h2 id="r-feat"></h2><div id="r-step" style="color:#C9D3D1;margin-top:4px"></div></div>' +
        `${milestones(s)}${notice(s)}</div>` +
        '<div class="side"><span class="muted" id="r-etal"></span><span class="big" id="r-eta"></span>' +
        '<span class="muted small" id="r-loc"></span><span class="amber-text small" id="r-hold"></span>' +
        (hold ? '<div style="margin-top:auto">' +
          cmdButton('secondary', 'go_home', 'Go home', 'Stops the mode, returns home') + '</div>' : '') +
        '</div>',
      actions: hold
        ? cmdButton('primary', 'hold_skip', 'False alarm · resume', 'No holds until this loop ends',
                    {accent: true})
        : cmdButton('primary', 'go_home', 'Go home', 'Stops the mode, returns home'),
    };
  },

  update(root, s) {
    setText(root, '#r-mode', s.mode_label + (s.loop ? ` · loop ${s.loop_count}` : ''));
    const label = s.playlist_labels[s.index] || '';
    setText(root, '#r-feat', label);
    setText(root, '#r-step', s.step_label + (s.step_is_arm ? ' (placeholder)' : ''));
    setText(root, '#r-etal', s.loop ? 'Loop restarts in' : 'Done in');
    setText(root, '#r-eta', s.eta_s < 0 ? '–' : fmtTime(s.eta_s) + (s.eta_plus_teleop ? ' +' : ''));
    setText(root, '#r-loc', s.location_label + (s.eta_plus_teleop ? ' · plus a teleop session' : ''));
    setText(root, '#r-hold', {off_loop: 'Visitor hold off until this loop ends.',
                              off: 'Visitor hold off (Health screen to turn it back on).'}[s.hold_check] || '');
  },
};

export function runningTitle(s) {
  return esc(s.playlist_labels[s.index] || '');
}
