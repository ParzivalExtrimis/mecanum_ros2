// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Ended: save the session as training data, or discard it. While the arm is still stowing
// (state "stowing") both buttons wait.

import {cmdButton, esc, fmtTime, notice} from './common.js';

export const ended = {
  key(s) {
    return ['ended', s.state, s.ended_reason, s.demo_count].join('|');
  },

  html(s) {
    const stowing = s.state === 'stowing';
    return {
      content: `<div class="main"><h2>Teleop ended</h2><span class="muted">${esc(s.ended_reason)}. ` +
        `${stowing ? 'Arm stowing…' : 'Arm stowed, base braked.'}</span>` +
        '<div class="stats">' +
        `<div class="stat"><span class="muted small">Run</span><span class="v">#${s.demo_count + 1}</span></div>` +
        `<div class="stat"><span class="muted small">Length</span><span class="v">${fmtTime(s.ended_length_s)}</span></div>` +
        `</div></div><div class="side">${notice(s)}</div>`,
      actions: cmdButton('split-a', 'discard_run', 'Discard', 'Not used for training') +
        cmdButton('split-b', 'save_run', 'Save run', `Counter goes to ${s.demo_count + 1}`,
                  {accent: true}),
    };
  },

  update() {},
};
