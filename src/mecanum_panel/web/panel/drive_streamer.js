// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Drive streamer: the 20 Hz Twist on /panel/drive while a D-pad key is held, only in Manual
// driving. The supervisor passes it on only in Manual with a fresh heartbeat and zeroes the
// base 200 ms after the last message, so a frozen panel cannot leave the base driving.

import {TYPES} from '../common/rosbridge.js';

// Requested speeds; the supervisor caps them again (drive_max_*).
const SPEED = {linear: 0.15, angular: 0.5};
const KEYS = {
  f: [1, 0, 0], b: [-1, 0, 0], l: [0, 1, 0], r: [0, -1, 0], ccw: [0, 0, 1], cw: [0, 0, -1],
};

export class DriveStreamer {
  constructor(ros, store) {
    this.ros = ros;
    this.store = store;
    this.held = new Set();
    ros.advertise('/panel/drive', TYPES.twist);
    setInterval(() => this._tick(), 50);
  }

  active() {
    const s = this.store.state;
    return this.store.linkUp() && s.state === 'manual' && s.manual_sub === 'drive';
  }

  press(key) {
    if (KEYS[key]) this.held.add(key);
    this._tick();
  }

  release(key) {
    if (key === undefined) this.held.clear();
    else this.held.delete(key);
    this._tick();
  }

  _tick() {
    if (this.held.size === 0) return;
    if (!this.active()) {
      this.held.clear();
      return;
    }
    let vx = 0;
    let vy = 0;
    let wz = 0;
    for (const key of this.held) {
      vx += KEYS[key][0];
      vy += KEYS[key][1];
      wz += KEYS[key][2];
    }
    this.ros.publish('/panel/drive', {
      linear: {x: vx * SPEED.linear, y: vy * SPEED.linear, z: 0},
      angular: {x: 0, y: 0, z: wz * SPEED.angular},
    });
  }
}
