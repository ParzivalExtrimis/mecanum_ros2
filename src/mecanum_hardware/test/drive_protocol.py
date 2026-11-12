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
Python reference of the drive serial protocol (docs/hardware_drive_test.md, "Protocol").

Third implementation beside the ESP32 sketch and the C++ host (drive_protocol.cpp). It is used
by the fake ESP32 of the launch test and to generate the byte vectors of the C++ unit test.
"""

import struct

PROTOCOL_VERSION = 1

HELLO = 0x01
HELLO_REQ = 0x02
CONFIG = 0x10
CONFIG_ACK = 0x11
CMD = 0x20
STATE = 0x30

FLAG_ENABLE = 0x01
FLAG_CLEAR_FAULT = 0x02

FAULT_FE_WHEEL0 = 0x0001       # bits 0-3: following error per wheel (latched)
FAULT_WATCHDOG = 0x0010
FAULT_NO_CONFIG = 0x0020
FAULT_OVERSPEED = 0x0040
FAULT_LATCHED = 0x0080
FAULT_STEP_DRIVER = 0x0100

MODE_NO_CONFIG = 0
MODE_IDLE = 1
MODE_RUNNING = 2
MODE_FAULT = 3

BODY_FORMATS = {
    HELLO: '<HBBBII',          # protocol, fw major, fw minor, wheels, default ppr, default cpr
    HELLO_REQ: '<',
    CONFIG: '<II6BHffi',       # ppr, cpr, motor_for_wheel[4], dir_inv, enc_inv, watchdog ms,
                               # accel backstop rad/s^2, max wheel speed rad/s, fe limit counts
    CONFIG_ACK: '<B',          # 0 ok, 1 invalid, 2 rejected while moving
    CMD: '<HB4f',              # seq, flags, wheel velocity rad/s x4 (FL, FR, RL, RR)
    STATE: '<HHQ4q4q4iHHB',    # seq, last cmd seq, mcu us, counts x4, pulses x4, fe x4,
                               # faults, crc errors, mode
}


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    block = bytearray()
    for byte in data:
        if byte == 0:
            out.append(len(block) + 1)
            out += block
            block = bytearray()
        else:
            block.append(byte)
            if len(block) == 254:
                out.append(255)
                out += block
                block = bytearray()
    out.append(len(block) + 1)
    out += block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            raise ValueError('bad COBS frame')
        out += data[i + 1:i + code]
        i += code
        if code < 255 and i < len(data):
            out.append(0)
    return bytes(out)


def encode(msg_type: int, *fields) -> bytes:
    """One framed message: COBS(type, body, crc16 LE) followed by 0x00."""
    payload = bytes([msg_type]) + struct.pack(BODY_FORMATS[msg_type], *fields)
    payload += struct.pack('<H', crc16(payload))
    return cobs_encode(payload) + b'\x00'


def decode(frame: bytes):
    """Decode one frame without its 0x00 delimiter: (type, fields) or raise ValueError."""
    payload = cobs_decode(frame)
    if len(payload) < 3:
        raise ValueError('short frame')
    body, crc = payload[:-2], struct.unpack('<H', payload[-2:])[0]
    if crc16(body) != crc:
        raise ValueError('bad CRC')
    msg_type = body[0]
    fmt = BODY_FORMATS.get(msg_type)
    if fmt is None or struct.calcsize(fmt) != len(body) - 1:
        raise ValueError('unknown type or size')
    return msg_type, struct.unpack(fmt, body[1:])


class FrameReader:
    """Splits a byte stream on 0x00 into decoded messages; counts bad frames."""

    def __init__(self):
        self._buf = bytearray()
        self.errors = 0

    def feed(self, data: bytes):
        messages = []
        self._buf += data
        while b'\x00' in self._buf:
            frame, _, rest = bytes(self._buf).partition(b'\x00')
            self._buf = bytearray(rest)
            if not frame:
                continue
            try:
                messages.append(decode(frame))
            except ValueError:
                self.errors += 1
        return messages
