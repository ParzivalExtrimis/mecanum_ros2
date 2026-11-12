// Copyright 2026 Aryan Hegde
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// mecanum_drive_esp32: the drive firmware of the mecanum base.
//
// An ESP32-DevKitC (WROOM-32E) between the host (ros2_control plugin mecanum_hardware/Esp32System,
// over USB serial) and four Leadshine CS-D808 closed-loop stepper drivers (pulse and direction).
// It reads each motor's encoder through the encoder tap (decision 0006) on the ESP32 pulse
// counter units, counts the step pulses it emits, and stops all wheels when the two disagree
// (the following error).
//
// The serial protocol is specified in docs/hardware_drive_test.md, section "Protocol". This
// file, the host (src/mecanum_hardware/src/drive_protocol.cpp) and the Python reference
// (src/mecanum_hardware/test/drive_protocol.py) implement it; PROTOCOL_VERSION must match.
//
// Arduino-ESP32 core 3.x (ESP-IDF 5). Built and uploaded by the owner; see README.md beside this
// file for the steps.
//
// Sections:
//   1. CONFIG            pins and defaults: the only part to edit
//   2. Pure logic        no Arduino types: CRC, COBS, conversions, following error, ramps
//   3. Protocol          message layouts, frame reader, senders
//   4. Encoders          pulse counter units, x4 decoding, 64-bit counts
//   5. Step generation   FastAccelStepper (RMT) or the built-in timer generator
//   6. Drive state       configuration, watchdog, fault latch
//   7. setup() / loop()

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "soc/gpio_reg.h"

// =================================================================================================
// 1. CONFIG
// =================================================================================================

// Firmware version, reported in HELLO.
#define FW_MAJOR 0
#define FW_MINOR 1
// Must equal PROTOCOL_VERSION in the host plugin. Change only with the protocol table.
#define PROTOCOL_VERSION 1

// Step generator: 1 = FastAccelStepper on the RMT peripheral (needs the FastAccelStepper library);
// 0 = the built-in timer generator of this file (no library). Both behave the same on the wire.
// Switch to 0 if FastAccelStepper does not build or misbehaves.
#ifndef STEP_DRIVER_FASTACCEL
#define STEP_DRIVER_FASTACCEL 1
#endif

// Pin map, owner's notes 2026-11-11. Index = motor 1..4 (index 0..3). Which motor drives which
// wheel is set at runtime by the host (motor_for_wheel in CONFIG), found on blocks (stage T2).
// STAND-IN: motor 3 DIR (5) and motor 4 PUL (4) were overwritten in the original notes; verify.
// GPIO 5 and 15 are strapping pins: harmless as DIR outputs, but check that the board still
// boots with the drivers connected (stage T1).
static const uint8_t PIN_PUL[4] = {18, 21, 23, 4};
static const uint8_t PIN_DIR[4] = {19, 22, 5, 15};
// Encoder A/B through the tap. 34/35 are input-only with no internal pull-up: the tap's level
// converter drives them (owner's side).
static const uint8_t PIN_ENC_A[4] = {34, 33, 26, 14};
static const uint8_t PIN_ENC_B[4] = {35, 32, 25, 27};

// Defaults until the host's CONFIG arrives (the host's values win). STAND-INs.
#define DEFAULT_PULSES_PER_REV 1600   // CS-D808 DIP setting: measured in stage T2
#define DEFAULT_COUNTS_PER_REV 4000   // 1000-line encoder, x4 decoding, direct drive

// The serial link to the host (the DevKitC's USB-UART bridge on UART0).
#define SERIAL_BAUD 921600
// STATE period: 100 Hz.
#define STATE_PERIOD_US 10000
// Direction setup before the first pulse after a direction change. The CS-D808 needs at least
// 5 us; 20 us leaves margin. (FastAccelStepper: dir_change_delay; timer generator: one or more
// control ticks, always longer.)
#define DIR_SETUP_US 20
// Encoder glitch filter: pulses shorter than this are ignored. At most 12700 ns (1023 APB
// cycles). 1000 ns is far below an encoder edge spacing at full speed (about 160 us per channel
// at 1 m/s).
#define ENC_GLITCH_NS 1000
// Hard limit on any pulse rate, whatever the host asks (Hz). The timer generator cannot exceed
// TIMER_TICK_HZ / 2.
#define MAX_PULSE_RATE_HZ 20000
// Below this rate a wheel is stopped (Hz).
#define MIN_PULSE_RATE_HZ 2
// Timer generator only: tick rate (Hz) and the ramp update period (us).
#define TIMER_TICK_HZ 40000
#define CONTROL_PERIOD_US 1000

#if STEP_DRIVER_FASTACCEL
#include <FastAccelStepper.h>
#endif

// =================================================================================================
// 2. Pure logic (no Arduino types; reviewed against the host tests)
// =================================================================================================

namespace logic
{

static const double kTwoPi = 6.283185307179586;

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
// Check value: "123456789" -> 0x29B1.
uint16_t crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// COBS encode `len` bytes into `out` (room for len + len / 254 + 1). Returns the encoded length,
// without the 0x00 delimiter.
size_t cobsEncode(const uint8_t * in, size_t len, uint8_t * out)
{
  size_t code_pos = 0;
  size_t o = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < len; ++i) {
    if (in[i] == 0) {
      out[code_pos] = code;
      code_pos = o++;
      code = 1;
    } else {
      out[o++] = in[i];
      ++code;
      if (code == 0xFF) {
        out[code_pos] = code;
        code_pos = o++;
        code = 1;
      }
    }
  }
  out[code_pos] = code;
  return o;
}

// COBS decode. Returns the decoded length, or 0 for a malformed frame.
size_t cobsDecode(const uint8_t * in, size_t len, uint8_t * out)
{
  size_t i = 0;
  size_t o = 0;
  while (i < len) {
    const uint8_t code = in[i];
    if (code == 0 || i + code > len) {
      return 0;
    }
    for (uint8_t k = 1; k < code; ++k) {
      out[o++] = in[i + k];
    }
    i += code;
    if (code < 0xFF && i < len) {
      out[o++] = 0;
    }
  }
  return o;
}

// Wheel speed (rad/s) to a signed pulse rate (Hz).
double wheelToPulseHz(double rad_s, uint32_t pulses_per_rev)
{
  return rad_s * static_cast<double>(pulses_per_rev) / kTwoPi;
}

// Wheel acceleration (rad/s^2) to pulses/s^2.
double wheelAccelToPulse(double rad_s2, uint32_t pulses_per_rev)
{
  return rad_s2 * static_cast<double>(pulses_per_rev) / kTwoPi;
}

// Following error in encoder counts: the emitted pulses scaled to counts, minus the counted
// position, minus the offset taken at the last configuration or fault clear.
int64_t followingError(
  int64_t pulses, int64_t counts, uint32_t pulses_per_rev, uint32_t counts_per_rev,
  int64_t offset)
{
  const int64_t expected = pulses * static_cast<int64_t>(counts_per_rev) /
    static_cast<int64_t>(pulses_per_rev);
  return expected - counts - offset;
}

// Move `current` toward `target` by at most `max_step`.
double rampToward(double current, double target, double max_step)
{
  if (target > current + max_step) {
    return current + max_step;
  }
  if (target < current - max_step) {
    return current - max_step;
  }
  return target;
}

// Clamp to +-limit; sets *clamped when it bites.
double clampAbs(double value, double limit, bool * clamped)
{
  if (value > limit) {
    *clamped = true;
    return limit;
  }
  if (value < -limit) {
    *clamped = true;
    return -limit;
  }
  return value;
}

// True when `elapsed` is past `timeout` (unsigned arithmetic, wraps safely).
bool expired(uint32_t now, uint32_t since, uint32_t timeout)
{
  return static_cast<uint32_t>(now - since) > timeout;
}

// Is motor_for_wheel a permutation of 0..3?
bool validMotorOrder(const uint8_t * order)
{
  bool seen[4] = {false, false, false, false};
  for (int w = 0; w < 4; ++w) {
    if (order[w] > 3 || seen[order[w]]) {
      return false;
    }
    seen[order[w]] = true;
  }
  return true;
}

}  // namespace logic

// =================================================================================================
// 3. Protocol (docs/hardware_drive_test.md, "Protocol"). Little-endian, packed.
// =================================================================================================

enum MsgType : uint8_t
{
  MSG_HELLO = 0x01,
  MSG_HELLO_REQ = 0x02,
  MSG_CONFIG = 0x10,
  MSG_CONFIG_ACK = 0x11,
  MSG_CMD = 0x20,
  MSG_STATE = 0x30,
};

enum : uint8_t
{
  FLAG_ENABLE = 0x01,
  FLAG_CLEAR_FAULT = 0x02,
};

enum : uint16_t
{
  FAULT_FE_WHEEL0 = 0x0001,   // bits 0..3: following error on wheel 0..3 (latched)
  FAULT_WATCHDOG = 0x0010,    // no CMD within the watchdog: wheels ramping to zero
  FAULT_NO_CONFIG = 0x0020,   // no valid CONFIG yet: nothing moves
  FAULT_OVERSPEED = 0x0040,   // a command was clamped to the maximum speed
  FAULT_LATCHED = 0x0080,     // a fault is latched: only zero until cleared
  FAULT_STEP_DRIVER = 0x0100, // the step generator failed to start (latched)
};

enum : uint8_t
{
  MODE_NO_CONFIG = 0,
  MODE_IDLE = 1,
  MODE_RUNNING = 2,
  MODE_FAULT = 3,
};

#pragma pack(push, 1)
struct HelloBody
{
  uint16_t protocol_version;
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t wheels;
  uint32_t default_pulses_per_rev;
  uint32_t default_counts_per_rev;
};
struct ConfigBody
{
  uint32_t pulses_per_rev;
  uint32_t counts_per_rev;
  uint8_t motor_for_wheel[4];   // motor index (0..3) of FL, FR, RL, RR
  uint8_t dir_invert;           // bit m: invert motor m's direction
  uint8_t enc_invert;           // bit m: invert motor m's encoder count
  uint16_t watchdog_ms;
  float accel_backstop;         // rad/s^2 at the wheel
  float max_wheel_speed;        // rad/s
  int32_t following_error_limit;  // counts; 0 = off
};
struct ConfigAckBody
{
  uint8_t status;  // 0 ok, 1 invalid, 2 rejected while moving
};
struct CmdBody
{
  uint16_t seq;
  uint8_t flags;
  float wheel_velocity[4];  // rad/s, FL, FR, RL, RR; positive rolls the robot forward
};
struct StateBody
{
  uint16_t seq;
  uint16_t last_cmd_seq;
  uint64_t mcu_time_us;
  int64_t counts[4];          // per wheel, in the wheel convention
  int64_t pulses[4];          // emitted, per wheel, in the wheel convention
  int32_t following_error[4]; // counts
  uint16_t faults;
  uint16_t crc_errors;
  uint8_t mode;
};
#pragma pack(pop)

static_assert(sizeof(HelloBody) == 13, "HELLO layout");
static_assert(sizeof(ConfigBody) == 28, "CONFIG layout");
static_assert(sizeof(CmdBody) == 19, "CMD layout");
static_assert(sizeof(StateBody) == 97, "STATE layout");

static uint16_t g_crc_errors = 0;

static void sendFrame(uint8_t type, const void * body, size_t len)
{
  uint8_t payload[1 + sizeof(StateBody) + 2];
  payload[0] = type;
  memcpy(payload + 1, body, len);
  const uint16_t crc = logic::crc16(payload, 1 + len);
  payload[1 + len] = static_cast<uint8_t>(crc & 0xFF);
  payload[2 + len] = static_cast<uint8_t>(crc >> 8);
  uint8_t frame[sizeof(payload) + 4];
  const size_t n = logic::cobsEncode(payload, len + 3, frame);
  frame[n] = 0x00;
  // Never block the loop: drop the frame if the TX buffer is full (the host sees a seq gap).
  if (Serial.availableForWrite() >= static_cast<int>(n + 1)) {
    Serial.write(frame, n + 1);
  }
}

static void sendHello()
{
  HelloBody b;
  b.protocol_version = PROTOCOL_VERSION;
  b.fw_major = FW_MAJOR;
  b.fw_minor = FW_MINOR;
  b.wheels = 4;
  b.default_pulses_per_rev = DEFAULT_PULSES_PER_REV;
  b.default_counts_per_rev = DEFAULT_COUNTS_PER_REV;
  sendFrame(MSG_HELLO, &b, sizeof(b));
}

static void sendConfigAck(uint8_t status)
{
  ConfigAckBody b{status};
  sendFrame(MSG_CONFIG_ACK, &b, sizeof(b));
}

// =================================================================================================
// 4. Encoders: one pulse counter unit per motor, x4 quadrature, extended to 64 bits.
// =================================================================================================

static pcnt_unit_handle_t g_pcnt[4] = {nullptr, nullptr, nullptr, nullptr};
static int g_pcnt_last[4] = {0, 0, 0, 0};
static int64_t g_enc_raw[4] = {0, 0, 0, 0};  // per motor, as counted (before any invert)
static bool g_enc_ok = true;

static bool encoderInit(int m)
{
  pcnt_unit_config_t unit_config = {};
  unit_config.low_limit = -30000;
  unit_config.high_limit = 30000;
  unit_config.flags.accum_count = 1;  // keep counting past the limits
  if (pcnt_new_unit(&unit_config, &g_pcnt[m]) != ESP_OK) {
    return false;
  }
  pcnt_glitch_filter_config_t filter_config = {};
  filter_config.max_glitch_ns = ENC_GLITCH_NS;
  pcnt_unit_set_glitch_filter(g_pcnt[m], &filter_config);

  // x4 decoding: both edges of A (level of B) and both edges of B (level of A), as in the
  // ESP-IDF rotary encoder example.
  pcnt_chan_config_t a_config = {};
  a_config.edge_gpio_num = PIN_ENC_A[m];
  a_config.level_gpio_num = PIN_ENC_B[m];
  pcnt_channel_handle_t chan_a = nullptr;
  if (pcnt_new_channel(g_pcnt[m], &a_config, &chan_a) != ESP_OK) {
    return false;
  }
  pcnt_chan_config_t b_config = {};
  b_config.edge_gpio_num = PIN_ENC_B[m];
  b_config.level_gpio_num = PIN_ENC_A[m];
  pcnt_channel_handle_t chan_b = nullptr;
  if (pcnt_new_channel(g_pcnt[m], &b_config, &chan_b) != ESP_OK) {
    return false;
  }
  pcnt_channel_set_edge_action(
    chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(
    chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(
    chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(
    chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  // accum_count needs watch points at the limits.
  pcnt_unit_add_watch_point(g_pcnt[m], unit_config.high_limit);
  pcnt_unit_add_watch_point(g_pcnt[m], unit_config.low_limit);
  if (pcnt_unit_enable(g_pcnt[m]) != ESP_OK || pcnt_unit_clear_count(g_pcnt[m]) != ESP_OK ||
    pcnt_unit_start(g_pcnt[m]) != ESP_OK)
  {
    return false;
  }
  return true;
}

// Call often (every loop): folds the counter into the 64-bit count.
static void encoderPoll()
{
  for (int m = 0; m < 4; ++m) {
    if (!g_pcnt[m]) {
      continue;
    }
    int value = 0;
    if (pcnt_unit_get_count(g_pcnt[m], &value) == ESP_OK) {
      g_enc_raw[m] += static_cast<int32_t>(static_cast<uint32_t>(value) -
        static_cast<uint32_t>(g_pcnt_last[m]));
      g_pcnt_last[m] = value;
    }
  }
}

// =================================================================================================
// 5. Step generation. Interface: stepInit(), stepSetTarget(motor, signed Hz, Hz/s),
//    stepPoll() every loop, stepPulses(motor) (signed, emitted), stepStill(motor).
//    Signs here are motor signs (after the per-motor direction invert).
// =================================================================================================

#if STEP_DRIVER_FASTACCEL

static FastAccelStepperEngine g_engine;
static FastAccelStepper * g_stepper[4] = {nullptr, nullptr, nullptr, nullptr};
static int32_t g_fas_last[4] = {0, 0, 0, 0};
static int64_t g_fas_pulses[4] = {0, 0, 0, 0};
static double g_fas_target[4] = {0, 0, 0, 0};

static bool stepInit()
{
  g_engine.init();
  for (int m = 0; m < 4; ++m) {
    g_stepper[m] = g_engine.stepperConnectToPin(PIN_PUL[m], DRIVER_RMT);
    if (!g_stepper[m]) {
      return false;
    }
    g_stepper[m]->setDirectionPin(PIN_DIR[m], true, DIR_SETUP_US);
    g_stepper[m]->setAcceleration(1000);
  }
  return true;
}

static void stepSetTarget(int m, double hz, double accel_hz_s)
{
  FastAccelStepper * s = g_stepper[m];
  if (!s) {
    return;
  }
  const uint32_t accel = accel_hz_s < 1.0 ? 1 : static_cast<uint32_t>(accel_hz_s);
  const double mag = fabs(hz);
  if (hz == g_fas_target[m]) {
    return;  // unchanged: let the running ramp continue
  }
  g_fas_target[m] = hz;
  s->setAcceleration(accel);
  if (mag < MIN_PULSE_RATE_HZ) {
    s->stopMove();  // decelerates at the acceleration set above
    return;
  }
  s->setSpeedInHz(static_cast<uint32_t>(mag));
  if (hz > 0) {
    s->runForward();
  } else {
    s->runBackward();
  }
}

static void stepPoll()
{
  for (int m = 0; m < 4; ++m) {
    if (!g_stepper[m]) {
      continue;
    }
    const int32_t pos = g_stepper[m]->getCurrentPosition();
    g_fas_pulses[m] += static_cast<int32_t>(static_cast<uint32_t>(pos) -
      static_cast<uint32_t>(g_fas_last[m]));
    g_fas_last[m] = pos;
  }
}

static int64_t stepPulses(int m) {return g_fas_pulses[m];}

static bool stepStill(int m) {return !g_stepper[m] || !g_stepper[m]->isRunning();}

#else  // the built-in timer generator

// A 40 kHz timer interrupt runs one phase accumulator per motor: each overflow emits one pulse
// (high for one tick). The loop ramps each rate toward its target every CONTROL_PERIOD_US and
// changes direction only at zero rate, one control tick after setting DIR (>= DIR_SETUP_US).
static hw_timer_t * g_timer = nullptr;
static portMUX_TYPE g_step_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t g_inc[4] = {0, 0, 0, 0};
static volatile uint32_t g_phase[4] = {0, 0, 0, 0};
static volatile int8_t g_dir[4] = {1, 1, 1, 1};
static volatile int64_t g_pulses[4] = {0, 0, 0, 0};
static volatile uint32_t g_high_mask = 0;
static uint32_t g_pul_mask[4];
static double g_rate[4] = {0, 0, 0, 0};       // current signed rate (Hz)
static double g_target[4] = {0, 0, 0, 0};
static double g_accel[4] = {1000, 1000, 1000, 1000};
static bool g_dir_pending[4] = {false, false, false, false};
static uint32_t g_last_control_us = 0;

static void ARDUINO_ISR_ATTR onStepTick()
{
  portENTER_CRITICAL_ISR(&g_step_mux);
  if (g_high_mask) {
    REG_WRITE(GPIO_OUT_W1TC_REG, g_high_mask);
    g_high_mask = 0;
  }
  uint32_t mask = 0;
  for (int m = 0; m < 4; ++m) {
    const uint32_t inc = g_inc[m];
    if (inc) {
      const uint32_t before = g_phase[m];
      const uint32_t after = before + inc;
      g_phase[m] = after;
      if (after < before) {
        mask |= g_pul_mask[m];
        g_pulses[m] += g_dir[m];
      }
    }
  }
  if (mask) {
    REG_WRITE(GPIO_OUT_W1TS_REG, mask);
    g_high_mask = mask;
  }
  portEXIT_CRITICAL_ISR(&g_step_mux);
}

static bool stepInit()
{
  for (int m = 0; m < 4; ++m) {
    pinMode(PIN_PUL[m], OUTPUT);
    digitalWrite(PIN_PUL[m], LOW);
    pinMode(PIN_DIR[m], OUTPUT);
    digitalWrite(PIN_DIR[m], HIGH);  // HIGH = positive, as FastAccelStepper's dirHighCountsUp
    g_pul_mask[m] = 1UL << PIN_PUL[m];  // all PUL pins are below 32
  }
  g_timer = timerBegin(1000000);  // 1 MHz timer clock
  if (!g_timer) {
    return false;
  }
  timerAttachInterrupt(g_timer, &onStepTick);
  timerAlarm(g_timer, 1000000 / TIMER_TICK_HZ, true, 0);
  g_last_control_us = micros();
  return true;
}

static void stepSetTarget(int m, double hz, double accel_hz_s)
{
  g_target[m] = hz;
  g_accel[m] = accel_hz_s < 1.0 ? 1.0 : accel_hz_s;
}

static void setIncrement(int m, double hz)
{
  const double inc = fabs(hz) * 4294967296.0 / TIMER_TICK_HZ;
  const uint32_t value = inc >= 4294967295.0 ? 0xFFFFFFFFu : static_cast<uint32_t>(inc);
  portENTER_CRITICAL(&g_step_mux);
  g_inc[m] = fabs(hz) < MIN_PULSE_RATE_HZ ? 0 : value;
  portEXIT_CRITICAL(&g_step_mux);
}

static void stepPoll()
{
  const uint32_t now = micros();
  const uint32_t dt_us = now - g_last_control_us;
  if (dt_us < CONTROL_PERIOD_US) {
    return;
  }
  g_last_control_us = now;
  const double dt = dt_us * 1e-6;
  for (int m = 0; m < 4; ++m) {
    if (g_dir_pending[m]) {
      g_dir_pending[m] = false;  // DIR was set one control tick ago: pulses may start
    }
    double next = logic::rampToward(g_rate[m], g_target[m], g_accel[m] * dt);
    const int8_t want = next > 0 ? 1 : (next < 0 ? -1 : g_dir[m]);
    if (want != g_dir[m]) {
      // Pass through zero, set DIR, and wait a control tick before pulsing the other way.
      next = 0.0;
      portENTER_CRITICAL(&g_step_mux);
      g_inc[m] = 0;
      g_dir[m] = want;
      portEXIT_CRITICAL(&g_step_mux);
      digitalWrite(PIN_DIR[m], want > 0 ? HIGH : LOW);
      g_dir_pending[m] = true;
    }
    g_rate[m] = next;
    if (!g_dir_pending[m]) {
      setIncrement(m, next);
    }
  }
}

static int64_t stepPulses(int m)
{
  portENTER_CRITICAL(&g_step_mux);
  const int64_t value = g_pulses[m];
  portEXIT_CRITICAL(&g_step_mux);
  return value;
}

static bool stepStill(int m) {return fabs(g_rate[m]) < MIN_PULSE_RATE_HZ;}

#endif

// =================================================================================================
// 6. Drive state
// =================================================================================================

struct RuntimeConfig
{
  uint32_t pulses_per_rev = DEFAULT_PULSES_PER_REV;
  uint32_t counts_per_rev = DEFAULT_COUNTS_PER_REV;
  uint8_t motor_for_wheel[4] = {0, 1, 2, 3};
  uint8_t dir_invert = 0;
  uint8_t enc_invert = 0;
  uint16_t watchdog_ms = 200;
  float accel_backstop = 5.0f;     // rad/s^2
  float max_wheel_speed = 3.0f;    // rad/s, about 0.3 m/s at the rim
  int32_t following_error_limit = 0;
};

static RuntimeConfig g_cfg;
static bool g_configured = false;
static bool g_step_ok = false;
static uint16_t g_latched = 0;        // latched fault bits
static bool g_enabled = false;
static uint16_t g_last_cmd_seq = 0;
static uint32_t g_last_cmd_ms = 0;
static bool g_watchdog = false;
static bool g_overspeed = false;
static float g_cmd[4] = {0, 0, 0, 0};  // wheel rad/s, FL FR RL RR
static int64_t g_fe_offset[4] = {0, 0, 0, 0};
static uint16_t g_state_seq = 0;
static uint32_t g_last_state_us = 0;

// Per wheel, in the wheel convention.
static int64_t wheelCounts(int w)
{
  const int m = g_cfg.motor_for_wheel[w];
  return (g_cfg.enc_invert >> m) & 1 ? -g_enc_raw[m] : g_enc_raw[m];
}

static int64_t wheelPulses(int w)
{
  const int m = g_cfg.motor_for_wheel[w];
  return (g_cfg.dir_invert >> m) & 1 ? -stepPulses(m) : stepPulses(m);
}

static int64_t wheelFollowingError(int w)
{
  return logic::followingError(
    wheelPulses(w), wheelCounts(w), g_cfg.pulses_per_rev, g_cfg.counts_per_rev, g_fe_offset[w]);
}

static bool allStill()
{
  for (int m = 0; m < 4; ++m) {
    if (!stepStill(m)) {
      return false;
    }
  }
  return true;
}

static void rebaseFollowingError()
{
  for (int w = 0; w < 4; ++w) {
    g_fe_offset[w] = 0;
    g_fe_offset[w] = wheelFollowingError(w);
  }
}

static void handleConfig(const ConfigBody & c)
{
  if (!allStill()) {
    sendConfigAck(2);
    return;
  }
  if (c.pulses_per_rev < 200 || c.pulses_per_rev > 51200 || c.counts_per_rev < 100 ||
    c.counts_per_rev > 1000000 || !logic::validMotorOrder(c.motor_for_wheel) ||
    c.watchdog_ms < 20 || c.watchdog_ms > 2000 || !(c.accel_backstop > 0.0f) ||
    !(c.max_wheel_speed > 0.0f) || c.following_error_limit < 0)
  {
    sendConfigAck(1);
    return;
  }
  g_cfg.pulses_per_rev = c.pulses_per_rev;
  g_cfg.counts_per_rev = c.counts_per_rev;
  memcpy(g_cfg.motor_for_wheel, c.motor_for_wheel, 4);
  g_cfg.dir_invert = c.dir_invert;
  g_cfg.enc_invert = c.enc_invert;
  g_cfg.watchdog_ms = c.watchdog_ms;
  g_cfg.accel_backstop = c.accel_backstop;
  g_cfg.max_wheel_speed = c.max_wheel_speed;
  g_cfg.following_error_limit = c.following_error_limit;
  g_configured = true;
  g_enabled = false;
  for (int w = 0; w < 4; ++w) {
    g_cmd[w] = 0.0f;
  }
  rebaseFollowingError();
  sendConfigAck(0);
}

static void handleCmd(const CmdBody & c)
{
  g_last_cmd_seq = c.seq;
  g_last_cmd_ms = millis();
  g_watchdog = false;
  if ((c.flags & FLAG_CLEAR_FAULT) && g_latched && allStill() && g_step_ok) {
    g_latched = 0;
    rebaseFollowingError();
  }
  g_enabled = (c.flags & FLAG_ENABLE) != 0;
  bool clamped = false;
  for (int w = 0; w < 4; ++w) {
    const float v = c.wheel_velocity[w];
    g_cmd[w] = isfinite(v) ?
      static_cast<float>(logic::clampAbs(v, g_cfg.max_wheel_speed, &clamped)) : 0.0f;
  }
  g_overspeed = clamped;
}

static void handleFrame(const uint8_t * frame, size_t len)
{
  uint8_t payload[64];
  if (len > sizeof(payload)) {
    ++g_crc_errors;
    return;
  }
  const size_t n = logic::cobsDecode(frame, len, payload);
  if (n < 3) {
    ++g_crc_errors;
    return;
  }
  const uint16_t crc = static_cast<uint16_t>(payload[n - 2] | (payload[n - 1] << 8));
  if (logic::crc16(payload, n - 2) != crc) {
    ++g_crc_errors;
    return;
  }
  const uint8_t type = payload[0];
  const uint8_t * body = payload + 1;
  const size_t body_len = n - 3;
  if (type == MSG_HELLO_REQ && body_len == 0) {
    sendHello();
  } else if (type == MSG_CONFIG && body_len == sizeof(ConfigBody)) {
    ConfigBody c;
    memcpy(&c, body, sizeof(c));
    handleConfig(c);
  } else if (type == MSG_CMD && body_len == sizeof(CmdBody)) {
    CmdBody c;
    memcpy(&c, body, sizeof(c));
    handleCmd(c);
  } else {
    ++g_crc_errors;
  }
}

// Serial frame reader: bytes up to 0x00 make one frame.
static uint8_t g_rx[96];
static size_t g_rx_len = 0;
static bool g_rx_overflow = false;

static void readSerial()
{
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) {
      break;
    }
    if (c == 0) {
      if (!g_rx_overflow && g_rx_len > 0) {
        handleFrame(g_rx, g_rx_len);
      } else if (g_rx_overflow) {
        ++g_crc_errors;
      }
      g_rx_len = 0;
      g_rx_overflow = false;
    } else if (g_rx_len < sizeof(g_rx)) {
      g_rx[g_rx_len++] = static_cast<uint8_t>(c);
    } else {
      g_rx_overflow = true;
    }
  }
}

// Apply the drive logic: what each motor should do now.
static void updateDrive()
{
  if (g_configured && g_enabled && !g_latched &&
    logic::expired(millis(), g_last_cmd_ms, g_cfg.watchdog_ms))
  {
    g_watchdog = true;
  }
  const bool may_move = g_configured && g_step_ok && g_enabled && !g_latched && !g_watchdog;

  // Following error: latch, and every wheel ramps to zero.
  if (g_configured && g_cfg.following_error_limit > 0) {
    for (int w = 0; w < 4; ++w) {
      const int64_t fe = wheelFollowingError(w);
      if (fe > g_cfg.following_error_limit || fe < -g_cfg.following_error_limit) {
        g_latched |= static_cast<uint16_t>(FAULT_FE_WHEEL0 << w) | FAULT_LATCHED;
      }
    }
  }

  const double accel = logic::wheelAccelToPulse(g_cfg.accel_backstop, g_cfg.pulses_per_rev);
  const double cap = MAX_PULSE_RATE_HZ;
  for (int w = 0; w < 4; ++w) {
    const int m = g_cfg.motor_for_wheel[w];
    double hz = may_move && !g_latched ?
      logic::wheelToPulseHz(g_cmd[w], g_cfg.pulses_per_rev) : 0.0;
    bool clamped = false;
    hz = logic::clampAbs(hz, cap, &clamped);
    if (clamped) {
      g_overspeed = true;
    }
    if ((g_cfg.dir_invert >> m) & 1) {
      hz = -hz;
    }
    if (g_configured && g_step_ok) {
      stepSetTarget(m, hz, accel);
    }
  }
}

static void sendState()
{
  StateBody s;
  s.seq = g_state_seq++;
  s.last_cmd_seq = g_last_cmd_seq;
  s.mcu_time_us = static_cast<uint64_t>(esp_timer_get_time());
  uint16_t faults = g_latched;
  for (int w = 0; w < 4; ++w) {
    s.counts[w] = wheelCounts(w);
    s.pulses[w] = wheelPulses(w);
    const int64_t fe = wheelFollowingError(w);
    s.following_error[w] = fe > INT32_MAX ? INT32_MAX : (fe < INT32_MIN ? INT32_MIN :
      static_cast<int32_t>(fe));
  }
  if (g_watchdog) {
    faults |= FAULT_WATCHDOG;
  }
  if (!g_configured) {
    faults |= FAULT_NO_CONFIG;
  }
  if (g_overspeed) {
    faults |= FAULT_OVERSPEED;
  }
  if (!g_step_ok || !g_enc_ok) {
    faults |= FAULT_STEP_DRIVER | FAULT_LATCHED;
  }
  s.faults = faults;
  s.crc_errors = g_crc_errors;
  if (!g_configured) {
    s.mode = MODE_NO_CONFIG;
  } else if (faults & FAULT_LATCHED) {
    s.mode = MODE_FAULT;
  } else if (g_enabled && !g_watchdog) {
    s.mode = MODE_RUNNING;
  } else {
    s.mode = MODE_IDLE;
  }
  sendFrame(MSG_STATE, &s, sizeof(s));
}

// =================================================================================================
// 7. setup() / loop()
// =================================================================================================

void setup()
{
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);

  // Steps first, so the PUL pins are driven low before anything else.
  g_step_ok = stepInit();
  for (int m = 0; m < 4; ++m) {
    if (!encoderInit(m)) {
      g_enc_ok = false;
    }
  }
  if (!g_step_ok || !g_enc_ok) {
    g_latched |= FAULT_STEP_DRIVER | FAULT_LATCHED;
  }
  delay(100);
  sendHello();
  g_last_state_us = micros();
}

void loop()
{
  readSerial();
  encoderPoll();
  stepPoll();
  updateDrive();
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - g_last_state_us) >= STATE_PERIOD_US) {
    g_last_state_us = now;
    sendState();
  }
}
