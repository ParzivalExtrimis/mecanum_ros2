// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Home: the only screen where a mode can be chosen. Tapping a tile or a chip selects it;
// the action button sends it, so a stray tap never starts the robot.

import {cmdButton, esc, setText, settingSeg} from './common.js';

const MODES = [
  ['1', 'Show loop', 'Runs on its own. The arm replay is a placeholder.'],
  ['2', 'Go home', 'Returns to the home tape.'],
  ['3', 'Manual', 'Staff drive the base with the D-pad.'],
  ['4', 'Show loop + teleop', 'Same loop; a visitor does the clean step.'],
  ['5', 'Teleop in place', 'Brakes on. Visitors drive the arm.'],
];

function feature(s, ui) {
  return s.features.find((f) => f.name === ui.selFeature) || null;
}

function hint(s, ui) {
  const f = feature(s, ui);
  if (f) {
    if (!f.ready) return {text: f.reason + '.', warn: true};
    const arm = f.arm ? ' Arm feature: a placeholder until the arm is integrated.' : '';
    return {text: (f.loopable ? 'Ready. This feature can loop.' : 'Ready. Runs once.') + arm,
            warn: false};
  }
  if (ui.selMode) return {text: MODES.find((m) => m[0] === ui.selMode)[2], warn: false};
  return {text: `${s.location_label}${s.pose_valid ? '' : ' · no position yet'}. ` +
          'Choose a mode or one feature.', warn: !s.pose_valid};
}

export const home = {
  key(s, ui) {
    const f = feature(s, ui);
    return ['home', ui.selMode, ui.selFeature, ui.loop, f && f.ready, s.hold_check,
            s.features.map((x) => x.ready).join()].join('|');
  },

  html(s, ui) {
    const tiles = MODES.map(([n, t, d]) =>
      `<button class="tile${ui.selMode === n ? ' sel' : ''}" data-ui="mode" data-v="${n}">` +
      `<span class="n">${n}</span><span class="t">${esc(t)}</span><span class="d">${esc(d)}</span>` +
      '</button>').join('');
    const chips = s.features.map((f) =>
      `<button class="chip${f.name === ui.selFeature ? ' sel' : ''}${f.ready ? '' : ' na'}" ` +
      `data-ui="feature" data-v="${f.name}">${esc(f.label)}` +
      `${f.arm ? '<span class="arm">arm</span>' : ''}</button>`).join('');
    const f = feature(s, ui);
    const canLoop = f && f.loopable;
    const seg = `<div class="seg"><button class="${!ui.loop ? 'on' : ''}" data-ui="loop" data-v="0">` +
      `Once</button><button class="${ui.loop ? 'on' : ''}" data-ui="loop" data-v="1"` +
      `${canLoop ? '' : ' disabled'}>Loop</button></div>`;
    let action;
    if (f) {
      action = cmdButton('primary', 'run_feature', `Run ${f.label}`, ui.loop && canLoop ? 'Loops until STOP' : 'Once',
                         {args: [f.name, ui.loop && canLoop ? 'loop' : 'once'], off: !f.ready,
                          accent: true});
    } else if (ui.selMode) {
      action = cmdButton('primary', 'start_mode', `Start mode ${ui.selMode}`,
                         MODES.find((m) => m[0] === ui.selMode)[1], {args: [ui.selMode], accent: true});
    } else {
      action = cmdButton('primary', 'start_mode', 'Choose a mode', '', {off: true});
    }
    return {
      content: `<div class="full"><div class="tiles">${tiles}</div>` +
        `<div class="chips">${chips}</div>` +
        `<div class="row" style="align-items:center">${seg}<span class="grow small" id="hint"></span>` +
        `<span class="small muted">Visitor hold</span>${settingSeg('hold_check', s.hold_check !== 'off')}</div></div>`,
      actions: action,
    };
  },

  update(root, s, ui) {
    const h = hint(s, ui);
    setText(root, '#hint', h.text);
    const el = root.querySelector('#hint');
    if (el) el.className = 'grow small ' + (h.warn ? 'amber-text' : 'muted');
  },
};
