// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// The booth TV: what the robot is doing, for visitors. It reads the same /supervisor/state as
// the panel, plus /map, /scan, /tf_static and /demo/plan, and draws on a 1280 x 720 canvas:
// the demonstration counter and status tag, the playlist, the booth map with the robot, its
// lidar returns and the path ahead, a live lidar view, and a caption. The wrist camera's
// panel is a placeholder until the arm is integrated.

import {bridgeUrl, fmtTime, Ros, TYPES} from '../common/rosbridge.js';

const W = 1280;
const H = 720;
const FOOT = {front: 0.421, rear: 0.405, half: 0.258};
// The map area shown: the booth and the aisle in front of it (map frame, metres).
const VIEW = {x0: -1.35, x1: 1.35, y0: -1.75, y1: 1.25};
const C = {
  bg: '#0B1012', text: '#F2F5F4', muted: '#9AA8A6', panel: '#111A1D', ring: '#2F4146',
  teal: '#1D9E75', mint: '#5DCAA5', amber: '#F2A93B', amberInk: '#2A1A00', red: '#E5484D',
  yellow: '#FAC775', blue: '#85B7EB', green: '#97C459', wall: '#5DCAA5', floor: '#1B282C',
};

const canvas = document.getElementById('tv');
const g = canvas.getContext('2d');
const ros = new Ros(bridgeUrl());
let state = null;
let stateAt = -Infinity;
let scan = null;
let plan = [];
let mapImage = null;
let mapInfo = null;
const statics = new Map();     // child frame -> {parent, x, y, yaw}
let lastCount = null;
let flashUntil = 0;

ros.subscribe('/supervisor/state', TYPES.state, (m) => {
  if (lastCount !== null && m.demo_count !== lastCount) flashUntil = performance.now() + 2000;
  lastCount = m.demo_count;
  state = m;
  stateAt = performance.now();
}, {queueLength: 1});
ros.subscribe('/scan', TYPES.scan, (m) => { scan = m; }, {throttleMs: 150, queueLength: 1});
ros.subscribe('/demo/plan', TYPES.path, (m) => {
  plan = m.poses.map((p) => [p.pose.position.x, p.pose.position.y]);
}, {queueLength: 1});
ros.subscribe('/map', TYPES.map, (m) => buildMap(m), {queueLength: 1});
ros.subscribe('/tf_static', TYPES.tf, (m) => {
  for (const t of m.transforms) {
    const q = t.transform.rotation;
    statics.set(t.child_frame_id, {
      parent: t.header.frame_id, x: t.transform.translation.x, y: t.transform.translation.y,
      yaw: Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))});
  }
});

function buildMap(m) {
  const {width, height} = m.info;
  const off = document.createElement('canvas');
  off.width = width;
  off.height = height;
  const ctx = off.getContext('2d');
  const img = ctx.createImageData(width, height);
  for (let row = 0; row < height; row++) {
    for (let col = 0; col < width; col++) {
      const v = m.data[row * width + col];
      const i = ((height - 1 - row) * width + col) * 4;   // row 0 is the bottom of the map
      const rgb = v >= 65 ? [93, 202, 165] : v < 0 ? [11, 16, 18] : [27, 40, 44];
      img.data.set([...rgb, 255], i);
    }
  }
  ctx.putImageData(img, 0, 0);
  mapImage = off;
  mapInfo = m.info;
}

// Pose of a frame relative to base_footprint, from the static transforms.
function framePose(frame) {
  let x = 0;
  let y = 0;
  let yaw = 0;
  let f = frame;
  for (let i = 0; i < 10 && f && f !== 'base_footprint'; i++) {
    const t = statics.get(f);
    if (!t) return null;
    // p_parent = t ∘ p_child
    const c = Math.cos(t.yaw);
    const s = Math.sin(t.yaw);
    [x, y] = [t.x + c * x - s * y, t.y + s * x + c * y];
    yaw += t.yaw;
    f = t.parent;
  }
  return f === 'base_footprint' ? {x, y, yaw} : null;
}

// Scan returns as [x, y] in the robot (base_footprint) frame.
function scanBody() {
  if (!scan) return [];
  const sensor = framePose(scan.header.frame_id) || {x: 0.4, y: 0, yaw: 0};
  const out = [];
  const c = Math.cos(sensor.yaw);
  const s = Math.sin(sensor.yaw);
  scan.ranges.forEach((r, i) => {
    if (r === null || !(r >= scan.range_min && r <= scan.range_max)) return;
    const a = scan.angle_min + i * scan.angle_increment;
    const lx = r * Math.cos(a);
    const ly = r * Math.sin(a);
    out.push([sensor.x + c * lx - s * ly, sensor.y + s * lx + c * ly]);
  });
  return out;
}

function rr(x, y, w, h, r) {
  g.beginPath();
  g.moveTo(x + r, y);
  g.arcTo(x + w, y, x + w, y + h, r);
  g.arcTo(x + w, y + h, x, y + h, r);
  g.arcTo(x, y + h, x, y, r);
  g.arcTo(x, y, x + w, y, r);
  g.closePath();
}

function pill(text, bg, fg, x, y, right = false, size = 22) {
  g.font = `600 ${size}px Inter, 'Noto Sans', sans-serif`;
  const w = g.measureText(text).width + 2 * size * 0.7;
  const left = right ? x - w : x;
  g.fillStyle = bg;
  rr(left, y, w, size * 1.9, 10);
  g.fill();
  g.fillStyle = fg;
  g.fillText(text, left + size * 0.7, y + size * 1.32);
}

const blocked = (s) => s.state === 'hold' || s.manual_blocked;

function tag(s) {
  if (s.state === 'estop') return ['E-stop', C.red, '#FFFFFF'];
  if (blocked(s)) return ['Waiting', C.amber, C.amberInk];
  if (s.state === 'live') return ['Live · human-guided', C.red, '#FFFFFF'];
  if (s.state === 'gate') return ['Starting soon', C.blue, '#042C53'];
  if (s.state === 'ended' || s.state === 'stowing') return ['Teleop ended', C.green, '#173404'];
  if (s.state === 'stopped' || s.state === 'stopping') return ['Paused', C.red, '#FFFFFF'];
  if (s.state === 'manual') return ['Manual control', '#D3D1C7', '#2C2C2A'];
  if (s.state === 'running') {
    if (s.feature === 'replay') return [`Replay of demo #${s.last_demo}`, C.yellow, C.amberInk];
    if (s.feature === 'greet') return ['Greeting', C.blue, '#042C53'];
    return ['Driving itself', C.teal, '#04211B'];
  }
  return [s.location === 'home' ? 'At home' : 'Idle', '#2F3D41', C.text];
}

function caption(s) {
  if (blocked(s)) return 'Someone came close. The robot waits.';
  if (s.state === 'running') {
    return s.feature === 'replay' ? 'Replaying a cleaning a person recorded earlier' : s.step_label;
  }
  return {
    gate: 'Next: a visitor at the counter guides the arm',
    live: 'A visitor at the counter is guiding the arm right now',
    stowing: 'Teleop finished. Arm stowing.',
    ended: 'Saving the run as training data',
    stopped: 'Paused by staff',
    stopping: 'Stopping',
    manual: 'Driven by staff',
    estop: 'Emergency stop pressed',
    idle: s.location === 'home' ? 'Ready at home' : 'Ready',
  }[s.state] || '';
}

function drawMap(x0, y0, w, h, s) {
  const scale = Math.min(w / (VIEW.x1 - VIEW.x0), h / (VIEW.y1 - VIEW.y0));
  const ox = x0 + (w - scale * (VIEW.x1 - VIEW.x0)) / 2;
  const oy = y0 + (h - scale * (VIEW.y1 - VIEW.y0)) / 2;
  const X = (x) => ox + (x - VIEW.x0) * scale;
  const Y = (y) => oy + (VIEW.y1 - y) * scale;
  g.save();
  g.beginPath();
  g.rect(X(VIEW.x0), Y(VIEW.y1), scale * (VIEW.x1 - VIEW.x0), scale * (VIEW.y1 - VIEW.y0));
  g.clip();
  g.fillStyle = C.floor;
  g.fillRect(X(VIEW.x0), Y(VIEW.y1), scale * (VIEW.x1 - VIEW.x0), scale * (VIEW.y1 - VIEW.y0));
  if (mapImage) {
    const res = mapInfo.resolution;
    const mx = mapInfo.origin.position.x;
    const my = mapInfo.origin.position.y;
    g.imageSmoothingEnabled = false;
    g.drawImage(mapImage, X(mx), Y(my + mapInfo.height * res),
                mapInfo.width * res * scale, mapInfo.height * res * scale);
  }
  const pose = s && s.pose_valid ? s.pose : null;
  if (pose && plan.length > 1 && (s.state === 'running' || s.state === 'hold')) {
    g.strokeStyle = C.yellow;
    g.lineWidth = 3;
    g.setLineDash([3, 6]);
    g.beginPath();
    plan.forEach(([px, py], i) => (i ? g.lineTo(X(px), Y(py)) : g.moveTo(X(px), Y(py))));
    g.stroke();
    g.setLineDash([]);
  }
  if (pose) {
    const c = Math.cos(pose.theta);
    const sn = Math.sin(pose.theta);
    g.fillStyle = blocked(s) ? '#F09595' : '#9FE1CB';
    for (const [bx, by] of scanBody()) {
      const mx = pose.x + c * bx - sn * by;
      const my = pose.y + sn * bx + c * by;
      g.fillRect(X(mx) - 2, Y(my) - 2, 4, 4);
    }
    g.save();
    g.translate(X(pose.x), Y(pose.y));
    g.rotate(-pose.theta);
    g.fillStyle = C.teal;
    g.fillRect(-FOOT.rear * scale, -FOOT.half * scale, (FOOT.front + FOOT.rear) * scale,
               2 * FOOT.half * scale);
    g.fillStyle = C.yellow;
    g.fillRect((FOOT.front - 0.08) * scale, -0.08 * scale, 0.08 * scale, 0.16 * scale);
    g.restore();
  }
  g.restore();
}

function drawLidar(x, y, w, h, s) {
  g.fillStyle = C.panel;
  rr(x, y, w, h, 12);
  g.fill();
  const cx = x + w / 2;
  const cy = y + h / 2;
  const k = h / 3.4;                 // pixels per metre: 1.7 m to the edge
  g.strokeStyle = C.ring;
  g.lineWidth = 1;
  for (const r of [0.5, 1.0, 1.5]) {
    g.beginPath();
    g.arc(cx, cy, r * k, 0, 2 * Math.PI);
    g.stroke();
  }
  g.fillStyle = blocked(s) ? '#F4C7C7' : C.mint;
  for (const [bx, by] of scanBody()) {
    const px = cx - by * k;          // robot's front points up
    const py = cy - bx * k;
    if (px > x + 4 && px < x + w - 4 && py > y + 4 && py < y + h - 4) {
      g.beginPath();
      g.arc(px, py, 3, 0, 2 * Math.PI);
      g.fill();
    }
  }
  g.fillStyle = C.yellow;
  g.beginPath();
  g.arc(cx, cy, 7, 0, 2 * Math.PI);
  g.fill();
  g.fillStyle = '#C9D3D1';
  g.font = "18px Inter, 'Noto Sans', sans-serif";
  g.fillText('Lidar, live', x + 16, y + h - 16);
}

function drawCamera(x, y, w, h, s) {
  g.fillStyle = '#262C2F';
  rr(x, y, w, h, 12);
  g.fill();
  g.fillStyle = C.muted;
  g.font = "600 26px Inter, 'Noto Sans', sans-serif";
  g.textAlign = 'center';
  g.fillText('Wrist camera', x + w / 2, y + h / 2 - 10);
  g.font = "20px Inter, 'Noto Sans', sans-serif";
  g.fillText('Not integrated yet: the arm is added later', x + w / 2, y + h / 2 + 24);
  g.textAlign = 'left';
  const t = {live: ['Live · human-guided', C.red, '#FFFFFF'], gate: ['Waiting for START', C.blue, '#042C53'],
             ended: ['Run finished', C.green, '#173404'], stowing: ['Run finished', C.green, '#173404']}[s.state]
    || (s.feature === 'greet' ? ['Greeting', C.blue, '#042C53']
      : [`Replay of demo #${s.last_demo}`, C.yellow, C.amberInk]);
  pill(t[0], t[1], t[2], x + 14, y + 14, false, 18);
}

function draw() {
  const now = performance.now();
  g.fillStyle = C.bg;
  g.fillRect(0, 0, W, H);
  const s = state && now - stateAt < 2000 ? state : null;
  if (!s) {
    g.fillStyle = C.muted;
    g.font = "600 40px Inter, 'Noto Sans', sans-serif";
    g.fillText(ros.connected ? 'Waiting for the robot…' : 'Connecting to the robot…', 60, H / 2);
    requestAnimationFrame(draw);
    return;
  }
  g.fillStyle = C.muted;
  g.font = "26px Inter, 'Noto Sans', sans-serif";
  g.fillText('Demonstrations recorded', 32, 60);
  g.fillStyle = now < flashUntil ? C.green : C.text;
  g.font = "600 46px 'Space Grotesk', Inter, 'Noto Sans', sans-serif";
  g.fillText(String(s.demo_count), 360, 64);
  const t = tag(s);
  pill(t[0], t[1], t[2], W - 32, 22, true, 24);

  let top = 100;
  const showPlaylist = s.playlist.length && !['idle', 'manual', 'estop'].includes(s.state)
    && s.mode !== '5' && s.mode !== '3';
  if (showPlaylist) {
    const n = s.playlist.length;
    const gw = (W - 64 - (n - 1) * 10) / n;
    const acc = {hold: C.amber, stopped: C.red, stopping: C.red}[s.state] || '#2BB39A';
    s.playlist_labels.forEach((label, i) => {
      const x = 32 + i * (gw + 10);
      g.fillStyle = i < s.index ? '#6F7D7B' : i === s.index ? acc : '#2A3539';
      rr(x, top, gw, 10, 5);
      g.fill();
      g.fillStyle = i === s.index ? C.text : C.muted;
      g.font = `${i === s.index ? '600 ' : ''}22px Inter, 'Noto Sans', sans-serif`;
      g.fillText(label, x, top + 40);
    });
    top += 64;
  }
  const bottom = 610;
  const camera = ['gate', 'live', 'stowing', 'ended'].includes(s.state) || s.step_is_arm
    || !s.arm_stowed;
  if (camera) {
    drawCamera(32, top, 760, bottom - top, s);
    g.fillStyle = C.muted;
    g.font = "20px Inter, 'Noto Sans', sans-serif";
    g.fillText('Booth map', 820, top + 20);
    drawMap(820, top + 30, 428, bottom - top - 30, s);
  } else {
    drawMap(32, top, 640, bottom - top, s);
    drawLidar(700, top, 548, bottom - top, s);
  }
  if (s.state === 'live') {
    g.fillStyle = C.text;
    g.font = "600 30px 'Space Grotesk', Inter, sans-serif";
    g.fillText(fmtTime(s.live_elapsed_s), 680, top + 46);
  }
  g.fillStyle = blocked(s) ? C.amber : '#FFFFFF';
  g.font = "500 36px Inter, 'Noto Sans', sans-serif";
  g.fillText(caption(s), 32, 670);
  if (blocked(s)) {
    g.fillStyle = C.amber;
    rr(W - 32 - 500, top + 16, 500, 170, 18);
    g.fill();
    g.fillStyle = C.amberInk;
    g.font = "600 52px 'Space Grotesk', Inter, sans-serif";
    g.fillText('Robot paused', W - 500, top + 90);
    g.font = "500 30px Inter, 'Noto Sans', sans-serif";
    g.fillText('Please step back a little', W - 500, top + 145);
  }
  if (s.state === 'estop') {
    g.fillStyle = 'rgba(90,16,18,0.88)';
    g.fillRect(0, 0, W, H);
    g.fillStyle = '#FFFFFF';
    g.font = "600 68px 'Space Grotesk', Inter, sans-serif";
    g.textAlign = 'center';
    g.fillText('Emergency stop', W / 2, H / 2 + 20);
    g.textAlign = 'left';
  }
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);
