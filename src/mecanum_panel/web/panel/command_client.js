// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Command client: one command in flight at a time. A second tap while one is pending is
// ignored, except STOP, which always goes out at once. A refused command is never retried;
// its reason is shown for a few seconds.

import {TYPES} from '../common/rosbridge.js';

export class CommandClient {
  constructor(ros) {
    this.ros = ros;
    this.pending = null;        // {action, id}
    this.error = '';
    this.errorUntil = 0;
    this.counter = 0;
    this.listeners = new Set();
  }

  onChange(callback) {
    this.listeners.add(callback);
  }

  _emit() {
    for (const cb of this.listeners) cb(this);
  }

  message() {
    return performance.now() < this.errorUntil ? this.error : '';
  }

  isPending(action) {
    return this.pending !== null && (action === undefined || this.pending.action === action);
  }

  send(action, args = []) {
    if (this.pending && action !== 'stop') return;
    const id = `panel-${Date.now()}-${++this.counter}`;
    const mine = {action, id};
    if (action !== 'stop' || !this.pending) this.pending = mine;
    this._emit();
    this.ros.callService('/supervisor/command', TYPES.command,
                         {action, args: args.map(String), request_id: id})
      .then((reply) => {
        if (!reply.accepted) this._fail(reply.reason || 'Refused');
      })
      .catch((err) => this._fail(err.message))
      .finally(() => {
        if (this.pending === mine) this.pending = null;
        this._emit();
      });
  }

  _fail(reason) {
    this.error = reason;
    this.errorUntil = performance.now() + 4000;
    this._emit();
    setTimeout(() => this._emit(), 4100);
  }
}
