// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// The panel app. State flows up: the ROS adapter, the store, the router, one screen.
// Requests flow down: a tap becomes a command, a drive stream or the heartbeat, and goes out
// the same websocket. No layer skips another (docs/Robot control panel app architecture.md).

import {bridgeUrl, diagLevel, LEVEL_DOT, Ros, TYPES} from '../common/rosbridge.js';
import {CommandClient} from './command_client.js';
import {DriveStreamer} from './drive_streamer.js';
import {startHeartbeat} from './heartbeat.js';
import {Router} from './router.js';
import {StateStore} from './state_store.js';
import {ended} from './screens/ended.js';
import {gate} from './screens/gate.js';
import {health} from './screens/health.js';
import {home} from './screens/home.js';
import {live} from './screens/live.js';
import {manual} from './screens/manual.js';
import {estopOverlay, holdOverlay, lockedOverlay} from './screens/overlays.js';
import {running} from './screens/running.js';
import {stopped} from './screens/stopped.js';

const SCREENS = {home, running, gate, live, ended, stopped, manual, health};
const $ = (id) => document.getElementById(id);

const fixture = new URLSearchParams(location.search).get('fixture');
const fx = fixture ? await import('./fixtures.js') : null;
const ros = fx ? new fx.FixtureRos() : new Ros(bridgeUrl());
const store = new StateStore();
const router = new Router(store);
const commands = new CommandClient(ros);
const drive = new DriveStreamer(ros, store);
startHeartbeat(ros);

ros.onConnection((up) => store.setConnected(up));
ros.subscribe('/supervisor/state', TYPES.state, (msg) => store.setState(msg), {queueLength: 1});
ros.subscribe('/robot/health', TYPES.health, (msg) => store.setHealth(msg), {queueLength: 1});
if (fx) {
  // Development: one sample state, refreshed as the robot would, and no websocket.
  const feed = () => {
    const s = fx.fixtureState(fixture);
    if (s) store.setState(s);
  };
  store.setHealth(fx.HEALTH);
  feed();
  setInterval(feed, 100);
  if (fixture === 'health') router.healthOpen = true;
}

// Panel-only selections on Home: what is selected, not what runs.
const ui = {selMode: null, selFeature: null, loop: false};

let shownKey = null;
let overlayKey = null;

function render() {
  router.tick();
  const view = router.view();
  $('linklost').classList.toggle('on', view.screen === null);
  renderStatus();
  if (view.screen === null) {
    shownKey = null;
    return;
  }
  const s = store.state;
  if (s.state !== 'idle') {
    ui.selMode = null;
    ui.selFeature = null;
  }
  const screen = SCREENS[view.screen];
  const key = view.screen + '#' + screen.key(s, ui, store);
  if (key !== shownKey) {
    drive.release();
    const out = screen.html(s, ui, store);
    $('content').innerHTML = out.content;
    $('actions').innerHTML = out.actions;
    shownKey = key;
  }
  screen.update($('frame'), s, ui, store);
  renderOverlays(view, s);
  for (const b of document.querySelectorAll('[data-cmd]')) {
    const action = b.dataset.cmd;
    const busy = commands.isPending() && action !== 'stop';
    b.disabled = !store.allowed(action) || b.dataset.off === '1' || busy;
    b.classList.toggle('pending', commands.isPending(action));
  }
}

function renderOverlays(view, s) {
  const parts = [];
  if (view.overlays.includes('hold')) parts.push(holdOverlay(s));
  if (view.overlays.includes('estop')) parts.push(estopOverlay());
  if (view.overlays.includes('locked')) parts.push(lockedOverlay(router.unlockProgress()));
  const html = parts.join('');
  if (html !== overlayKey) {
    $('overlay').innerHTML = html;
    overlayKey = html;
  }
}

function renderStatus() {
  const s = store.state;
  const up = store.linkUp();
  $('sb-mode').textContent = s ? s.mode_label : 'Connecting…';
  const moving = s && s.base_moving;
  $('sb-brake').innerHTML = `<i class="dot ${moving ? 'amber' : ''}"></i>${moving ? 'Moving' : 'Brakes on'}`;
  const armOut = s && !s.arm_stowed;
  $('sb-arm').innerHTML = `<i class="dot ${armOut ? 'blue' : 'grey'}"></i>` +
    (s && !s.arm_integrated ? 'Arm n/a' : armOut ? 'Arm out' : 'Arm stowed');
  const lidar = store.health && store.health.status.find((x) => x.name === '2D lidar');
  const level = lidar ? diagLevel(lidar.level) : 3;
  $('sb-lidar').innerHTML = `<i class="dot ${LEVEL_DOT[level] || 'grey'}"></i>Lidar`;
  const off = !s ? '' : s.state === 'manual' && !s.manual_checks ? 'Checks off'
    : {off_loop: 'Hold off · this loop', off: 'Hold off'}[s.hold_check] || '';
  $('sb-safe').innerHTML = off ? `<i class="dot amber"></i><span class="amber-text">${off}</span>` : '';
  $('sb-link').innerHTML = `<i class="dot ${up ? '' : 'red'}"></i>${up ? 'Linked' : 'No link'}`;
  const err = commands.message();
  $('sb-err').textContent = err;
  $('sb-err').classList.toggle('on', Boolean(err));
}

// ----- input ---------------------------------------------------------------------------

$('frame').addEventListener('pointerdown', (e) => {
  router.touch();
  const dir = e.target.closest('[data-dir]');
  if (dir && !dir.disabled) {
    dir.setPointerCapture(e.pointerId);
    dir.classList.add('held');
    drive.press(dir.dataset.dir);
  }
  if (e.target.closest('[data-unlock]')) router.unlockStart = performance.now();
});

function endPress(e) {
  const dir = e.target.closest && e.target.closest('[data-dir]');
  if (dir) {
    dir.classList.remove('held');
    drive.release(dir.dataset.dir);
  }
  if (router.unlockStart !== null && router.unlockProgress() < 1) router.unlockStart = null;
}
$('frame').addEventListener('pointerup', endPress);
$('frame').addEventListener('pointercancel', endPress);
$('frame').addEventListener('lostpointercapture', endPress);

$('frame').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (b && b.disabled) return;
  if (b && b.dataset.cmd) {
    const args = b.dataset.args ? b.dataset.args.split(',') : [];
    commands.send(b.dataset.cmd, args);
    render();
    return;
  }
  if (b && b.dataset.ui) {
    onUi(b.dataset.ui, b.dataset.v);
    render();
    return;
  }
  if (e.target.closest('#status') && !router.locked && store.linkUp()) {
    router.healthOpen = true;
    render();
  }
});

function onUi(name, value) {
  if (name === 'mode') {
    ui.selMode = ui.selMode === value ? null : value;
    ui.selFeature = null;
  } else if (name === 'feature') {
    ui.selFeature = ui.selFeature === value ? null : value;
    ui.selMode = null;
    const f = store.state.features.find((x) => x.name === ui.selFeature);
    if (!f || !f.loopable) ui.loop = false;
  } else if (name === 'loop') {
    ui.loop = value === '1';
  } else if (name === 'back') {
    router.healthOpen = false;
  }
}

// W A S D Q E drive in Manual, for development at a keyboard.
const KEYMAP = {w: 'f', s: 'b', a: 'l', d: 'r', q: 'ccw', e: 'cw'};
addEventListener('keydown', (e) => {
  const k = KEYMAP[e.key.toLowerCase()];
  if (k && !e.repeat) drive.press(k);
});
addEventListener('keyup', (e) => {
  const k = KEYMAP[e.key.toLowerCase()];
  if (k) drive.release(k);
});
addEventListener('blur', () => drive.release());

// ----- frame scaling -------------------------------------------------------------------

function fit() {
  const scale = new URLSearchParams(location.search).get('fit') === '0' ? 1
    : Math.min(innerWidth / 800, innerHeight / 480);
  $('frame').style.transform = `scale(${scale})`;
}
addEventListener('resize', fit);
fit();

store.onChange(() => render());
commands.onChange(() => render());
setInterval(render, 100);
render();
