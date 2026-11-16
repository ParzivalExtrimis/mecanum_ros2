// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Overlays sit on top of the screen underneath and leave STOP uncovered. Link lost is the
// exception: a STOP sent over a dead link would not arrive, so it covers everything.

import {esc} from './common.js';

export function holdOverlay(s) {
  const clear = s.person_close ? `Resumes on its own ${s.hold_clear_needed_s.toFixed(0)} s after the area clears.`
    : `Area clear. Resuming in ${Math.max(0, s.hold_clear_needed_s - s.hold_clear_s).toFixed(1)} s.`;
  return `<div class="ov-hold"><h2>Holding: someone is close</h2><span>${esc(clear)}</span></div>`;
}

export function estopOverlay() {
  return '<div class="ov-panel ov-estop"><h2>E-stop pressed</h2>' +
    '<span>Motor power is cut in hardware. The software cannot override it.</span>' +
    '<span>Twist the button to release it, then choose what to do.</span></div>' +
    '<div class="ov-actions"><button class="primary" disabled>Reset after release' +
    '<small>Available once the e-stop is released</small></button></div>';
}

export function lockedOverlay(progress) {
  const r = 90;
  const c = 2 * Math.PI * r;
  return '<div class="ov-panel ov-locked"><button class="unlock" data-unlock="1">' +
    `<svg viewBox="0 0 192 192"><circle cx="96" cy="96" r="${r}" ` +
    `stroke-dasharray="${c}" stroke-dashoffset="${c * (1 - progress)}"/></svg>` +
    'Hold 2 s<br>to unlock</button>' +
    '<span class="muted">Locked after a minute at home. STOP and the e-stop still work.</span></div>' +
    '<div class="ov-actions"></div>';
}
