// Copyright 2026 Aryan Hegde
// Licensed under the Apache License, Version 2.0.
//
// Heartbeat: 2 Hz on /panel/heartbeat whenever any screen is open. Missing for more than 1 s,
// the supervisor stops Manual driving.

import {TYPES} from '../common/rosbridge.js';

export function startHeartbeat(ros) {
  ros.advertise('/panel/heartbeat', TYPES.header);
  setInterval(() => {
    const now = Date.now();
    ros.publish('/panel/heartbeat', {
      stamp: {sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1e6},
      frame_id: 'panel',
    });
  }, 500);
}
