// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Live: one visitor session. The auto-end countdown stays amber so staff see a session about
// to time out.

import {cmdButton, fmtTime, notice, setText} from './common.js';

export const live = {
  key(s) {
    return ['live', Boolean(s.notice)].join('|');
  },

  html(s) {
    return {
      content: '<div class="main"><div class="banner b-live"><span style="font-weight:600">' +
        '<i class="dot red"></i><span id="l-title"></span></span><span class="mid" id="l-t"></span></div>' +
        '<div class="stats"><div class="stat"><span class="muted small">Auto-ends in</span>' +
        '<span class="v amber-text" id="l-idle"></span></div>' +
        '<div class="stat"><span class="muted small">Speed limit</span><span class="v">50%</span></div></div>' +
        `${notice(s)}</div>` +
        '<div class="side"><span class="muted">No leader motion for</span><div class="bar"><i id="l-bar"></i></div>' +
        '<span class="muted small">The base stays braked for the whole session.</span></div>',
      actions: cmdButton('primary', 'teleop_stop', 'End session', 'Arm holds, then stows'),
    };
  },

  update(root, s) {
    setText(root, '#l-title', `Live · recording demo #${s.demo_count + 1}`);
    setText(root, '#l-t', fmtTime(s.live_elapsed_s));
    const left = Math.max(0, s.live_idle_limit_s - s.live_idle_s);
    setText(root, '#l-idle', `${Math.ceil(left)} s`);
    const bar = root.querySelector('#l-bar');
    if (bar) bar.style.width = `${Math.min(100, 100 * s.live_idle_s / s.live_idle_limit_s)}%`;
  },
};
