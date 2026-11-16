// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Screen router: turns the robot's state into one screen plus overlays. The panel's own
// navigation is only two things: opening Health (a tap on the status bar) and the lock, which
// comes on after 60 s idle at home and goes off with a 2 s long-press. It never changes state
// on the robot.

export const LOCK_AFTER_MS = 60000;
export const UNLOCK_HOLD_MS = 2000;

const SCREEN_OF = {
  idle: 'home', running: 'running', hold: 'running', gate: 'gate', live: 'live',
  stowing: 'ended', ended: 'ended', stopping: 'stopped', stopped: 'stopped', manual: 'manual',
  estop: 'stopped',
};

export class Router {
  constructor(store) {
    this.store = store;
    this.healthOpen = false;
    this.locked = false;
    this.lastTouch = performance.now();
    this.idleSince = null;
    this.unlockStart = null;
  }

  touch() {
    this.lastTouch = performance.now();
  }

  unlockProgress() {
    if (this.unlockStart === null) return 0;
    return Math.min(1, (performance.now() - this.unlockStart) / UNLOCK_HOLD_MS);
  }

  tick() {
    const s = this.store.state;
    const now = performance.now();
    if (this.unlockStart !== null && this.unlockProgress() >= 1) {
      this.locked = false;
      this.unlockStart = null;
      this.touch();
    }
    if (!s || !(s.state === 'idle' && s.location === 'home')) {
      this.idleSince = null;
      this.locked = false;
      return;
    }
    if (this.idleSince === null) this.idleSince = now;
    if (!this.locked && now - Math.max(this.lastTouch, this.idleSince) > LOCK_AFTER_MS) {
      this.locked = true;
      this.healthOpen = false;
    }
  }

  view() {
    if (!this.store.linkUp()) return {screen: null, overlays: ['linklost']};
    const s = this.store.state;
    const overlays = [];
    if (s.state === 'hold') overlays.push('hold');
    if (s.state === 'estop') overlays.push('estop');
    if (this.locked) overlays.push('locked');
    return {screen: this.healthOpen ? 'health' : SCREEN_OF[s.state] || 'home', overlays};
  }
}
