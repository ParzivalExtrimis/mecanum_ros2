// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Health: one row per part of the robot, from /robot/health. The only screen the panel opens
// by itself (a tap on the status bar), without the robot changing state. It also holds the
// visitor-hold setting, so that it can be turned off mid-loop when false holds pile up.

import {diagLevel, LEVEL_DOT} from '../../common/rosbridge.js';
import {esc, settingSeg, uiButton} from './common.js';


export const health = {
  key(s, ui, store) {
    const h = store.health;
    return 'health|' + s.hold_check + '|' +
      (h ? h.status.map((x) => x.name + diagLevel(x.level) + x.message).join() : '');
  },

  html(s, ui, store) {
    const rows = store.health ? store.health.status.map((x) =>
      `<div class="hrow"><i class="dot ${LEVEL_DOT[diagLevel(x.level)] || 'grey'}"></i><span>${esc(x.name)}</span>` +
      `<span class="r">${esc(x.message)}</span></div>`).join('')
      : '<span class="muted">No health report from the robot yet.</span>';
    const setting = '<div class="hrow"><span>Visitor hold</span>' +
      `<span class="r">${{off_loop: 'off this loop', off: 'off'}[s.hold_check] || 'on'}</span>` +
      settingSeg('hold_check', s.hold_check !== 'off', ['On', 'Off'], 'seg mini') + '</div>';
    return {
      content: `<div class="full"><h2>Health</h2><div class="hlist">${rows}${setting}</div></div>`,
      actions: uiButton('primary', 'back', 'Back', 'To the current screen'),
    };
  },

  update() {},
};
