// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Shared pieces of the screens. A screen is {key, html, update}:
//   key(s, ui)        changes when the screen's structure must be rebuilt
//   html(s, ui)       {content, actions}: markup for the content area and the action band
//   update(root, s, ui)  refreshes the texts that change every tick, without rebuilding
// Buttons carry data-cmd (a supervisor action, with data-args) or data-ui (a panel-only
// action). The app enables a data-cmd button only when the robot lists its action, and never
// when data-off="1". Screens never call ROS, start timers or decide what is allowed.

import {esc, fmtTime} from '../../common/rosbridge.js';

export {esc, fmtTime};

export function cmdButton(cls, action, label, sub = '', {args = [], off = false, accent = false} = {}) {
  const a = args.length ? ` data-args="${esc(args.join(','))}"` : '';
  return `<button class="${cls}${accent ? ' accent' : ''}" data-cmd="${action}"${a}` +
    `${off ? ' data-off="1"' : ''}>${esc(label)}${sub ? `<small>${esc(sub)}</small>` : ''}</button>`;
}

export function uiButton(cls, name, label, sub = '', value = '') {
  return `<button class="${cls}" data-ui="${name}" data-v="${esc(value)}">${esc(label)}` +
    `${sub ? `<small>${esc(sub)}</small>` : ''}</button>`;
}

// An on/off setting the robot keeps (hold_check, manual_checks): two buttons, the current one lit.
export function settingSeg(action, on, labels = ['On', 'Off'], cls = 'seg') {
  return `<div class="${cls}">` +
    cmdButton(on ? 'on' : '', action, labels[0], '', {args: ['on']}) +
    cmdButton(on ? '' : 'on', action, labels[1], '', {args: ['off']}) + '</div>';
}

export function milestones(s) {
  const n = s.playlist.length;
  if (!n) return '';
  const tone = {hold: ' hold', stopped: ' stop', stopping: ' stop', estop: ' stop'}[s.state] || '';
  return `<div class="ms" style="grid-template-columns:repeat(${n},minmax(0,1fr))">` +
    s.playlist_labels.map((label, i) => {
      const cls = i < s.index ? 'done' : i === s.index ? 'now' + tone : '';
      return `<div><div class="b ${cls}"></div><div class="l${i === s.index ? ' now' : ''}">` +
        `${esc(label)}</div></div>`;
    }).join('') + '</div>';
}

export function notice(s) {
  return s.notice ? `<div class="banner b-info"><span class="s">${esc(s.notice)}</span></div>` : '';
}

export function setText(root, selector, text) {
  const el = root.querySelector(selector);
  if (el && el.textContent !== text) el.textContent = text;
}
