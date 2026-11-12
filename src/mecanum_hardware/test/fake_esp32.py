# Copyright 2026 Aryan Hegde
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
A fake drive ESP32 on a pseudo-terminal, for tests and rehearsals without the board.

It speaks the drive protocol (docs/hardware_drive_test.md, "Protocol") with the behaviour of
firmware/mecanum_drive_esp32: HELLO at start and on request, CONFIG checked and accepted only
with every wheel still, the watchdog, the acceleration backstop, the speed clamp, the latched
following-error fault and its clear, and STATE at 100 Hz. The wheels are ideal: each wheel's
encoder follows its emitted pulses exactly, in the wheel convention (the motor order and the
inverts are checked, then ignored, as for perfect wiring).

    python3 fake_esp32.py --link /tmp/fake_esp32
    ros2 launch mecanum_bringup drive_test.launch.py port:=/tmp/fake_esp32

Signals, to test the stop paths:
    SIGUSR1  toggle a frozen front-left encoder (a dead channel): the following error grows
             while that wheel turns, then the fault latches
    SIGUSR2  toggle a mute link (nothing sent, nothing read), as a pulled USB cable
    SIGTERM, SIGINT  exit and remove the link
"""

import argparse
import math
import os
import select
import signal
import sys
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import drive_protocol as dp  # noqa: E402

TWO_PI = 2.0 * math.pi
STATE_PERIOD = 0.01
TICK = 0.001
FW_MAJOR, FW_MINOR = 0, 1
MAX_PULSE_RATE_HZ = 20000.0
STILL_RAD_S = 1e-6


class FakeDrive:
    """The firmware's drive logic on ideal wheels, in the wheel convention."""

    def __init__(self, pulses_per_rev, counts_per_rev):
        self.ppr = pulses_per_rev
        self.cpr = counts_per_rev
        self.watchdog_s = 0.2
        self.accel = 5.0
        self.max_speed = 3.0
        self.fe_limit = 0
        self.configured = False
        self.enabled = False
        self.latched = 0
        self.watchdog = False
        self.overspeed = False
        self.last_cmd_seq = 0
        self.last_cmd_time = time.monotonic()
        self.cmd = [0.0] * 4
        self.rate = [0.0] * 4          # rad/s, after the backstop ramp
        self.angle = [0.0] * 4         # rad, the emitted pulses' angle
        self.counted = [0.0] * 4       # rad, what the encoder saw
        self.fe_offset = [0] * 4
        self.frozen = False
        self.crc_errors = 0
        self.state_seq = 0

    def pulses(self, w):
        return math.floor(self.angle[w] * self.ppr / TWO_PI)

    def counts(self, w):
        return math.floor(self.counted[w] * self.cpr / TWO_PI)

    def following_error(self, w):
        # As logic::followingError: C++ integer division truncates toward zero.
        p = self.pulses(w) * self.cpr
        expected = abs(p) // self.ppr * (1 if p >= 0 else -1)
        return expected - self.counts(w) - self.fe_offset[w]

    def all_still(self):
        return all(abs(r) < STILL_RAD_S for r in self.rate)

    def rebase(self):
        for w in range(4):
            self.fe_offset[w] = 0
            self.fe_offset[w] = self.following_error(w)

    def on_config(self, fields):
        ppr, cpr, m0, m1, m2, m3, _dir_inv, _enc_inv, watchdog_ms, accel, max_speed, fe = fields
        if not self.all_still():
            return 2
        if (not 200 <= ppr <= 51200 or not 100 <= cpr <= 1000000
                or sorted((m0, m1, m2, m3)) != [0, 1, 2, 3] or not 20 <= watchdog_ms <= 2000
                or not accel > 0.0 or not max_speed > 0.0 or fe < 0):
            return 1
        # Keep the wheel angles; the counts rescale, as the firmware's raw counts would not.
        self.ppr, self.cpr = ppr, cpr
        self.watchdog_s = watchdog_ms / 1000.0
        self.accel, self.max_speed, self.fe_limit = accel, max_speed, fe
        self.configured = True
        self.enabled = False
        self.cmd = [0.0] * 4
        self.rebase()
        return 0

    def on_cmd(self, fields, now):
        seq, flags, *velocity = fields
        self.last_cmd_seq = seq
        self.last_cmd_time = now
        self.watchdog = False
        if flags & dp.FLAG_CLEAR_FAULT and self.latched and self.all_still():
            self.latched = 0
            self.rebase()
        self.enabled = bool(flags & dp.FLAG_ENABLE)
        clamped = False
        for w, v in enumerate(velocity):
            if not math.isfinite(v):
                v = 0.0
            if abs(v) > self.max_speed:
                v = math.copysign(self.max_speed, v)
                clamped = True
            self.cmd[w] = v
        self.overspeed = clamped

    def step(self, now, dt):
        if (self.configured and self.enabled and not self.latched
                and now - self.last_cmd_time > self.watchdog_s):
            self.watchdog = True
        if self.configured and self.fe_limit > 0:
            for w in range(4):
                if abs(self.following_error(w)) > self.fe_limit:
                    self.latched |= (dp.FAULT_FE_WHEEL0 << w) | dp.FAULT_LATCHED
        may_move = self.configured and self.enabled and not self.latched and not self.watchdog
        cap = MAX_PULSE_RATE_HZ * TWO_PI / self.ppr
        for w in range(4):
            target = self.cmd[w] if may_move else 0.0
            if abs(target) > cap:
                target = math.copysign(cap, target)
                self.overspeed = True
            step = self.accel * dt
            self.rate[w] = max(self.rate[w] - step, min(self.rate[w] + step, target))
            if abs(self.rate[w]) < STILL_RAD_S:
                self.rate[w] = 0.0
            self.angle[w] += self.rate[w] * dt
            # A frozen channel loses the counts for good, as a real one would: after the
            # thaw the encoder moves on from where it stopped.
            if not (self.frozen and w == 0):
                self.counted[w] += self.rate[w] * dt

    def state_frame(self, now_us):
        faults = self.latched
        if self.watchdog:
            faults |= dp.FAULT_WATCHDOG
        if not self.configured:
            faults |= dp.FAULT_NO_CONFIG
        if self.overspeed:
            faults |= dp.FAULT_OVERSPEED
        if not self.configured:
            mode = dp.MODE_NO_CONFIG
        elif faults & dp.FAULT_LATCHED:
            mode = dp.MODE_FAULT
        elif self.enabled and not self.watchdog:
            mode = dp.MODE_RUNNING
        else:
            mode = dp.MODE_IDLE
        fe = [max(-2**31, min(2**31 - 1, self.following_error(w))) for w in range(4)]
        frame = dp.encode(
            dp.STATE, self.state_seq, self.last_cmd_seq, now_us,
            *[self.counts(w) for w in range(4)], *[self.pulses(w) for w in range(4)], *fe,
            faults, self.crc_errors & 0xFFFF, mode)
        self.state_seq = (self.state_seq + 1) & 0xFFFF
        return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--link', required=True,
                        help="Path of the symlink to the pty, the plugin's port parameter")
    parser.add_argument('--pulses-per-rev', type=int, default=1600,
                        help='Boot default, reported in HELLO (the host overrides it)')
    parser.add_argument('--counts-per-rev', type=int, default=4000)
    parser.add_argument('--pid-file', help='Write the process id here, for the signals')
    args = parser.parse_args()

    master, slave = os.openpty()
    # Raw, so 0x00 and control bytes pass untouched; the slave stays open here, so the master
    # keeps working while the host closes and reopens the port.
    tty.setraw(slave)
    os.set_blocking(master, False)
    if os.path.lexists(args.link):
        os.remove(args.link)
    os.symlink(os.ttyname(slave), args.link)
    if args.pid_file:
        with open(args.pid_file, 'w') as f:
            f.write(str(os.getpid()))

    drive = FakeDrive(args.pulses_per_rev, args.counts_per_rev)
    flags = {'mute': False, 'run': True}

    def toggle_freeze(*_):
        drive.frozen = not drive.frozen
        print(f'fake_esp32: front-left encoder {"frozen" if drive.frozen else "free"}',
              flush=True)

    def toggle_mute(*_):
        flags['mute'] = not flags['mute']
        print(f'fake_esp32: link {"muted" if flags["mute"] else "back"}', flush=True)

    def stop(*_):
        flags['run'] = False

    signal.signal(signal.SIGUSR1, toggle_freeze)
    signal.signal(signal.SIGUSR2, toggle_mute)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    reader = dp.FrameReader()
    start = time.monotonic()
    last = start
    next_state = start
    out = bytearray()

    def send(frame):
        out.extend(frame)

    send(dp.encode(dp.HELLO, dp.PROTOCOL_VERSION, FW_MAJOR, FW_MINOR, 4,
                   args.pulses_per_rev, args.counts_per_rev))
    print(f'fake_esp32: listening on {args.link} -> {os.ttyname(slave)}', flush=True)
    try:
        while flags['run']:
            try:
                readable, _, _ = select.select([master], [], [], TICK)
            except InterruptedError:
                readable = []
            now = time.monotonic()
            if readable:
                try:
                    data = os.read(master, 4096)
                except (BlockingIOError, OSError):
                    data = b''
                if data and not flags['mute']:
                    errors = reader.errors
                    for msg_type, fields in reader.feed(data):
                        if msg_type == dp.HELLO_REQ:
                            send(dp.encode(dp.HELLO, dp.PROTOCOL_VERSION, FW_MAJOR, FW_MINOR,
                                           4, drive.ppr, drive.cpr))
                        elif msg_type == dp.CONFIG:
                            send(dp.encode(dp.CONFIG_ACK, drive.on_config(fields)))
                        elif msg_type == dp.CMD:
                            drive.on_cmd(fields, now)
                        else:
                            drive.crc_errors += 1
                    drive.crc_errors += reader.errors - errors
            drive.step(now, now - last)
            last = now
            if now >= next_state:
                next_state += STATE_PERIOD
                if next_state < now:
                    next_state = now + STATE_PERIOD
                send(drive.state_frame(int((now - start) * 1e6)))
            if flags['mute']:
                out.clear()
            elif out:
                try:
                    written = os.write(master, bytes(out))
                    del out[:written]
                except BlockingIOError:
                    pass
                except OSError:
                    out.clear()
                # A host that stopped reading must not grow the queue without bound.
                if len(out) > 65536:
                    out.clear()
    finally:
        if os.path.islink(args.link):
            os.remove(args.link)
        if args.pid_file and os.path.exists(args.pid_file):
            os.remove(args.pid_file)


if __name__ == '__main__':
    main()
