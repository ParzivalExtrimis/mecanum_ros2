// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// State store: only what the robot last said. It keeps the latest /supervisor/state and
// /robot/health, when they arrived, and whether the link is up. It never keeps its own copy
// of mode or progress, so after a reconnect the panel shows whatever the supervisor reports.

export const LINK_TIMEOUT_MS = 1000;

export class StateStore {
  constructor() {
    this.state = null;
    this.health = null;
    this.receivedAt = -Infinity;
    this.connected = false;
    this.listeners = new Set();
  }

  onChange(callback) {
    this.listeners.add(callback);
  }

  _emit() {
    for (const cb of this.listeners) cb(this);
  }

  setState(msg) {
    this.state = msg;
    this.receivedAt = performance.now();
    this._emit();
  }

  setHealth(msg) {
    this.health = msg;
    this._emit();
  }

  setConnected(value) {
    this.connected = value;
    this._emit();
  }

  linkAgeMs() {
    return performance.now() - this.receivedAt;
  }

  linkUp() {
    return this.connected && this.state !== null && this.linkAgeMs() <= LINK_TIMEOUT_MS;
  }

  // Safety rule 1: a button is enabled only if the robot lists its action.
  allowed(action) {
    return this.linkUp() && this.state.allowed_actions.includes(action);
  }
}
