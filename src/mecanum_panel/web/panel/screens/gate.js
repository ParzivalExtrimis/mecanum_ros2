// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Teleop gate: the four checks, and START only when all of them pass. Mode 4 shows its
// fallback countdown above STOP.

import {cmdButton, esc, notice, setText} from './common.js';

export const gate = {
  key(s) {
    return ['gate', s.mode, s.checks.map((c) => c.ok).join(), Boolean(s.notice)].join('|');
  },

  html(s) {
    const checks = s.checks.map((c, i) =>
      `<div class="check"><span class="mark ${c.ok ? 'ok' : 'no'}">${c.ok ? '✓' : '•'}</span>` +
      `<span>${esc(c.name)}</span><span class="r" id="g-d${i}"></span></div>`).join('');
    const fallback = s.mode === '4';
    const other = fallback
      ? cmdButton('secondary', 'teleop_skip', 'Skip, replay instead', 'Automatic when the time runs out')
      : cmdButton('secondary', 'teleop_cancel', 'Cancel', s.mode === '3' ? 'Back to Manual' : 'End this mode');
    return {
      content: '<div class="main"><div class="row"><h2>Start teleoperation</h2>' +
        '<span class="muted small">Nothing moves until START</span></div>' +
        `<div style="display:flex;flex-direction:column;gap:12px">${checks}</div></div>` +
        `<div class="side">${fallback
          ? '<span class="muted">Replay starts in</span><span class="big amber-text" id="g-left"></span>'
          : '<span class="muted">Waiting as long as needed</span>'}` +
        `${notice(s)}<span class="grow"></span>${other}</div>`,
      actions: cmdButton('primary', 'teleop_start', 'START', 'Follower mirrors the leader',
                         {accent: true}),
    };
  },

  update(root, s) {
    s.checks.forEach((c, i) => setText(root, `#g-d${i}`, c.detail));
    if (s.gate_remaining_s >= 0) setText(root, '#g-left', `${Math.ceil(s.gate_remaining_s)} s`);
  },
};
