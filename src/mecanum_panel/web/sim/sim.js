// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Simulation-only controls: what the booth simulator's side panel did, through ROS. None of
// these exist on the robot: /sim/visitor (mecanum_simulation booth_visitor), /estop published
// from here (the real e-stop publishes it on hardware), and the supervisor's development switch.

import {bridgeUrl, Ros, TYPES} from '../common/rosbridge.js';

const $ = (id) => document.getElementById(id);
const ros = new Ros(bridgeUrl());
let visitorIn = false;
let estop = false;

ros.onConnection((up) => { $('link').textContent = up ? '· linked' : '· no link'; });
ros.advertise('/estop', TYPES.bool, true);
ros.subscribe('/sim/visitor_in', TYPES.bool, (m) => {
  visitorIn = m.data;
  $('visitor').textContent = visitorIn ? 'Visitor steps back' : 'Visitor leans in';
  $('visitor').classList.toggle('on', visitorIn);
});
ros.subscribe('/estop', TYPES.bool, (m) => {
  estop = m.data;
  $('estop').textContent = estop ? 'Release E-STOP' : 'E-STOP (hardware)';
  $('estop').classList.toggle('on', estop);
});

$('visitor').onclick = () => {
  ros.callService('/sim/visitor', TYPES.setBool, {data: !visitorIn}, 5000)
    .then((r) => log(r.message || 'Visitor moved'))
    .catch((e) => log('Visitor: ' + e.message));
};
$('estop').onclick = () => {
  ros.publish('/estop', {data: !estop});
  log(!estop ? 'E-STOP pressed' : 'E-STOP released');
};
$('arm').onchange = () => {
  ros.callService('/supervisor/dev/arm_checks_pass', TYPES.setBool, {data: $('arm').checked})
    .then((r) => log(r.message))
    .catch((e) => log('Arm switch: ' + e.message));
};

let last = '';
ros.subscribe('/supervisor/state', TYPES.state, (s) => {
  const line = `${s.state}${s.feature ? ' · ' + s.feature : ''}${s.step_label ? ' · ' + s.step_label : ''}` +
    `${s.message ? ' · ' + s.message : ''}`;
  if (line !== last) {
    last = line;
    log(line);
  }
}, {queueLength: 1});

const t0 = performance.now();
function log(text) {
  const s = Math.floor((performance.now() - t0) / 1000);
  const li = document.createElement('li');
  li.innerHTML = `<time>${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}</time><span></span>`;
  li.lastChild.textContent = text;
  $('log').prepend(li);
  while ($('log').children.length > 80) $('log').lastChild.remove();
}
