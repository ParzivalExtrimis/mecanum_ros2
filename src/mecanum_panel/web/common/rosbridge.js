// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// The ROS adapter: a small client for the rosbridge v2 protocol over one websocket.
// It owns the socket, the subscriptions, the advertisements and the reconnects, and never
// interprets a message. After a reconnect it subscribes and advertises again by itself.

export class Ros {
  constructor(url) {
    this.url = url;
    this.ws = null;
    this.connected = false;
    this.nextId = 1;
    this.subs = new Map();      // topic -> {type, options, callbacks: Set}
    this.ads = new Map();       // topic -> {type, latch}
    this.calls = new Map();     // id -> {resolve, reject, timer}
    this.listeners = new Set(); // (connected) => void
    this._open();
  }

  onConnection(callback) {
    this.listeners.add(callback);
    callback(this.connected);
    return () => this.listeners.delete(callback);
  }

  subscribe(topic, type, callback, options = {}) {
    let sub = this.subs.get(topic);
    if (!sub) {
      sub = {type, options, callbacks: new Set()};
      this.subs.set(topic, sub);
      this._send(this._subscribeOp(topic, sub));
    }
    sub.callbacks.add(callback);
    return () => {
      sub.callbacks.delete(callback);
      if (sub.callbacks.size === 0) {
        this.subs.delete(topic);
        this._send({op: 'unsubscribe', topic});
      }
    };
  }

  advertise(topic, type, latch = false) {
    if (this.ads.has(topic)) return;
    this.ads.set(topic, {type, latch});
    this._send({op: 'advertise', topic, type, latch});
  }

  publish(topic, msg) {
    if (!this.connected || !this.ads.has(topic)) return false;
    this._send({op: 'publish', topic, msg});
    return true;
  }

  // Resolves with the response values; rejects on a failed call, a lost link or a timeout.
  callService(service, type, args = {}, timeoutMs = 3000) {
    return new Promise((resolve, reject) => {
      if (!this.connected) {
        reject(new Error('Not connected to the robot'));
        return;
      }
      const id = `call_${this.nextId++}`;
      const timer = setTimeout(() => {
        this.calls.delete(id);
        reject(new Error('The robot did not answer'));
      }, timeoutMs);
      this.calls.set(id, {resolve, reject, timer});
      this._send({op: 'call_service', id, service, type, args});
    });
  }

  _subscribeOp(topic, sub) {
    const op = {op: 'subscribe', topic, type: sub.type};
    if (sub.options.throttleMs) op.throttle_rate = sub.options.throttleMs;
    if (sub.options.queueLength) op.queue_length = sub.options.queueLength;
    return op;
  }

  _send(op) {
    if (this.connected && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(op));
    }
  }

  _setConnected(value) {
    if (this.connected === value) return;
    this.connected = value;
    for (const cb of this.listeners) cb(value);
  }

  _open() {
    let ws;
    try {
      ws = new WebSocket(this.url);
    } catch (e) {
      setTimeout(() => this._open(), 1000);
      return;
    }
    this.ws = ws;
    ws.onopen = () => {
      this._setConnected(true);
      for (const [topic, sub] of this.subs) this._send(this._subscribeOp(topic, sub));
      for (const [topic, ad] of this.ads) {
        this._send({op: 'advertise', topic, type: ad.type, latch: ad.latch});
      }
    };
    ws.onmessage = (event) => {
      let data;
      try {
        data = JSON.parse(event.data);
      } catch (e) {
        return;
      }
      if (data.op === 'publish') {
        const sub = this.subs.get(data.topic);
        if (sub) for (const cb of sub.callbacks) cb(data.msg);
      } else if (data.op === 'service_response') {
        const call = this.calls.get(data.id);
        if (!call) return;
        clearTimeout(call.timer);
        this.calls.delete(data.id);
        if (data.result === false) call.reject(new Error(String(data.values || 'Service failed')));
        else call.resolve(data.values);
      }
    };
    ws.onclose = () => {
      this._setConnected(false);
      for (const [id, call] of this.calls) {
        clearTimeout(call.timer);
        call.reject(new Error('Lost the link to the robot'));
        this.calls.delete(id);
      }
      setTimeout(() => this._open(), 1000);
    };
    ws.onerror = () => ws.close();
  }
}

// The bridge's address: the host this page came from, on the configured port. A ?ws=... query
// parameter overrides it (for a panel served from elsewhere).
export function bridgeUrl() {
  const query = new URLSearchParams(location.search).get('ws');
  if (query) return query;
  const port = (window.DEMO_CONFIG && window.DEMO_CONFIG.wsPort) || 9090;
  const host = location.hostname || 'localhost';
  return `${location.protocol === 'https:' ? 'wss' : 'ws'}://${host}:${port}`;
}

export const TYPES = {
  state: 'mecanum_interfaces/msg/SupervisorState',
  command: 'mecanum_interfaces/srv/SupervisorCommand',
  twist: 'geometry_msgs/msg/Twist',
  header: 'std_msgs/msg/Header',
  health: 'diagnostic_msgs/msg/DiagnosticArray',
  bool: 'std_msgs/msg/Bool',
  setBool: 'std_srvs/srv/SetBool',
  map: 'nav_msgs/msg/OccupancyGrid',
  scan: 'sensor_msgs/msg/LaserScan',
  path: 'nav_msgs/msg/Path',
  tf: 'tf2_msgs/msg/TFMessage',
};

// Shared display names, so that the panel and the TV say the same thing.
export const STATE_WORDS = {
  idle: 'Idle', running: 'Running', hold: 'Holding', gate: 'Teleop gate', live: 'Live',
  stowing: 'Arm stowing', ended: 'Teleop ended', stopping: 'Stopping', stopped: 'Stopped',
  manual: 'Manual', estop: 'E-stop',
};

export function fmtTime(seconds) {
  const s = Math.max(0, Math.round(seconds));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
}

export function esc(text) {
  return String(text).replace(/[&<>"']/g, (c) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c]));
}

// A DiagnosticStatus level (0 OK, 1 WARN, 2 ERROR, 3 STALE) as a number. It is a ROS `byte`,
// which reaches the browser as a number, a base64 string or a one-element array depending on
// the bridge's version.
export function diagLevel(value) {
  if (typeof value === 'number') return value;
  if (Array.isArray(value)) return Number(value[0]);
  if (typeof value === 'string') {
    if (/^\d+$/.test(value)) return Number(value);
    try {
      return atob(value).charCodeAt(0);
    } catch (e) {
      return 3;
    }
  }
  return 3;
}

// The dot class for a level: green, amber, red, grey.
export const LEVEL_DOT = ['ok', 'amber', 'red', 'grey'];
