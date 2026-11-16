// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Stopped: by staff, by a fault or after an e-stop release. Reset is the main way on; Resume,
// Go home and Manual are the secondary ones. Also drawn while stopping (the arm stows) and
// under the e-stop overlay.

import {cmdButton, esc, milestones} from './common.js';

const TITLES = {staff: 'Stopped by staff', fault: 'Fault', estop: 'E-stop released'};

export const stopped = {
  key(s) {
    return ['stopped', s.state, s.stop_kind, s.message, s.location, s.index,
            s.playlist.join()].join('|');
  },

  html(s) {
    const title = s.state === 'stopping' ? 'Stopping' : s.state === 'estop' ? 'E-stop pressed'
      : TITLES[s.stop_kind] || 'Stopped';
    const detail = s.state === 'stopping' ? 'Arm lifting clear, then stowing. Base braked.'
      : `${s.location_label} · ${s.arm_stowed ? 'arm stowed' : 'arm out'} · brakes on`;
    return {
      content: `<div class="main"><div class="banner b-stop"><h2>${esc(title)}</h2>` +
        `${s.message ? `<span class="s">${esc(s.message)}</span>` : ''}` +
        `<span class="s">${esc(detail)}</span></div>${milestones(s)}</div>` +
        '<div class="side">' +
        cmdButton('secondary', 'resume', 'Resume', 'From where it stopped') +
        cmdButton('secondary', 'go_home', 'Go home', 'Then the mode list') +
        cmdButton('secondary', 'start_mode', 'Manual', 'Drive it clear', {args: ['3']}) +
        '</div>',
      actions: cmdButton('primary', 'reset', 'Reset', 'Home, then restart the mode', {accent: true}),
    };
  },

  update() {},
};
