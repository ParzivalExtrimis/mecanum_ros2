# mecanum_drive_esp32: drive firmware (owner builds and uploads)

One sketch, `mecanum_drive_esp32.ino`, for the ESP32-DevKitC (WROOM-32E) between the host and
the four CS-D808 stepper drivers. It generates the step pulses, counts the motor encoders
through the encoder tap ([decision 0006](../../docs/decisions/0006-wheel-state-from-tapped-encoders.md)),
and stops every wheel on a following error, a lost host or a bad configuration.

The host side is the ros2_control plugin `mecanum_hardware/Esp32System`. The serial protocol
is specified in [`docs/hardware_drive_test.md`](../../docs/hardware_drive_test.md), section
"Protocol", and the test stages that use this firmware are in the same document.

**This sketch is not compiled or uploaded by the stack's tooling: the owner does both.** The
steps below are everything the owner has to act on.

## 1. Install

1. **Board package:** Arduino-ESP32 core **3.x** (ESP-IDF 5). It was written against 3.3.12, the
   version installed on the dev laptop. Core 2.x will not build: the encoder code uses ESP-IDF
   5's `driver/pulse_cnt.h`.
2. **Library:** **FastAccelStepper** (by gin66), from the Library Manager, version **0.31 or
   newer** (the first with the RMT driver on ESP-IDF 5).
   - If it does not build, or misbehaves, skip it and use the built-in step generator: set
     `#define STEP_DRIVER_FASTACCEL 0` near the top of the sketch (section 1). No other change
     is needed; both generators behave the same on the wire.
3. Nothing else: the encoders use the core's built-in pulse counter driver.

## 2. Board options

| Option | Value |
|---|---|
| Board | **ESP32 Dev Module** |
| Core Debug Level | **None** (any log output shares the host's serial line) |
| Upload speed | 921600 (lower it if uploads fail) |
| Flash size, partition scheme, PSRAM | Defaults |
| Events Run On, Arduino Runs On | Core 1 (the defaults) |

## 3. Values to check in the CONFIG block (section 1 of the sketch)

Most values are only defaults: the host sends its own on every start (the CONFIG message, from
the ros2_control xacro), so **tuning never needs a re-upload.** Only the pins and the step
generator choice are fixed at build time.

| Item | Set to | How to find or check it |
|---|---|---|
| `PIN_PUL`, `PIN_DIR` | Motor 1..4: PUL 18, 21, 23, 4; DIR 19, 22, 5, 15 (your notes) | **Verify motor 3 DIR (5) and motor 4 PUL (4)**: your notes say they were overwritten. Follow the wires. |
| `PIN_ENC_A`, `PIN_ENC_B` | Motor 1..4: A 34, 33, 26, 14; B 35, 32, 25, 27 | From the tap's level converter outputs. 34/35 have no internal pull-up. |
| `DEFAULT_PULSES_PER_REV` | 1600 (STAND-IN) | The CS-D808 DIP switches (pulses per revolution table on the driver's label). Stage T2 measures it anyway. |
| `DEFAULT_COUNTS_PER_REV` | 4000 | 1000-line encoder × 4. Stage T2 measures it. |
| `STEP_DRIVER_FASTACCEL` | 1 | 0 if FastAccelStepper fails (step 1.2) |
| `DIR_SETUP_US` | 20 | The driver needs at least 5 µs from DIR to the next pulse |
| `ENC_GLITCH_NS` | 1000 | Raise toward 5000 only if counts jitter at standstill |
| `MAX_PULSE_RATE_HZ` | 20000 | A hard cap above any speed the host allows |

Motor-to-wheel order, direction and encoder inverts, the watchdog, the acceleration backstop,
the speed cap and the following-error limit all come from the host. There is nothing to edit
for them here.

## 4. After the upload: what to see

1. Close the Arduino serial monitor before starting ROS: only one program can hold the port.
2. With the serial monitor at **921600** baud, the board prints the boot ROM text at reset (at
   115200, so it shows as garbage), then binary frames: unreadable characters about 100 times a
   second. That is normal: it is the STATE stream. No readable text follows.
3. **The PUL pins stay quiet** after boot, and until the host has sent a valid CONFIG and a CMD
   with motion enabled. With the drivers powered, the motors hold torque but do not turn. The
   drivers' ENA is not wired, so they are enabled whenever they have power.
4. A log line like `gpio: gpio_pullup_en ... GPIO number error` at boot is harmless. The pulse
   counter driver tries to enable pull-ups, which GPIO 34/35 do not have. It is only visible
   with a Core Debug Level above None.
5. Find the stable port name for the xacro: `ls -l /dev/serial/by-id/`. The CP2102 or CH340
   bridge of the DevKitC appears there.

## 5. What each test stage may ask you to change

| Stage | Firmware change? |
|---|---|
| T1 (USB only, drivers unpowered) | Only if the board does not boot with the drivers connected: then move DIR off GPIO 5/15 and update `PIN_DIR` (re-upload). |
| T2 (on blocks) | None expected. Wheel order, signs and pulses per rev are host parameters. A wheel that never counts, or counts one way only, is the first sign of a tap fault (decision 0006, "Gaps"). |
| T3–T5 (floor) | None. All limits and ramps are host parameters. |
| Any | `STEP_DRIVER_FASTACCEL 0` if pulses stutter or stop with FastAccelStepper. |

After any re-upload, tell the stack which version is on the board: the host logs the
firmware version from HELLO when it connects.

## Behaviour summary

- **Stops** with every wheel ramped to zero at the acceleration backstop when any of these happens:
  - no CMD arrives within the watchdog (200 ms default), until the next CMD;
  - motion is disabled;
  - the following error passes its limit on any wheel (latched until a CMD with "clear fault"
    arrives while all wheels are still);
  - the step generator or the encoders failed to start (latched).
- **Reports** at 100 Hz:
  - the encoder counts and emitted pulses per wheel (64-bit, in the wheel convention), with the
    following error;
  - the fault bits and the mode;
  - the count of bad frames received.
- **Does not** close any speed loop. The CS-D808s close the position loop; the encoder counts
  are reported, and checked against the emitted pulses.
