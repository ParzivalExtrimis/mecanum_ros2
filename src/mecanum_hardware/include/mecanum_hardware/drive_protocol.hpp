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

#ifndef MECANUM_HARDWARE__DRIVE_PROTOCOL_HPP_
#define MECANUM_HARDWARE__DRIVE_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

/// The serial protocol between the host and the drive ESP32, as specified in
/// docs/hardware_drive_test.md, section "Protocol". The sketch
/// (firmware/mecanum_drive_esp32) and the Python reference (test/drive_protocol.py) implement
/// the same table; kProtocolVersion must match the sketch's PROTOCOL_VERSION.
///
/// A frame is COBS(type, body, CRC-16/CCITT-FALSE of type and body, little-endian) followed by
/// 0x00. All fields are little-endian.
namespace mecanum_hardware::drive_protocol
{

constexpr std::uint16_t kProtocolVersion = 1;
constexpr int kWheels = 4;

enum class MsgType : std::uint8_t
{
  kHello = 0x01,
  kHelloReq = 0x02,
  kConfig = 0x10,
  kConfigAck = 0x11,
  kCmd = 0x20,
  kState = 0x30,
};

constexpr std::uint8_t kFlagEnable = 0x01;
constexpr std::uint8_t kFlagClearFault = 0x02;

constexpr std::uint16_t kFaultFollowingErrorWheel0 = 0x0001;  // bits 0..3, latched
constexpr std::uint16_t kFaultWatchdog = 0x0010;
constexpr std::uint16_t kFaultNoConfig = 0x0020;
constexpr std::uint16_t kFaultOverspeed = 0x0040;
constexpr std::uint16_t kFaultLatched = 0x0080;
constexpr std::uint16_t kFaultStepDriver = 0x0100;

constexpr std::uint8_t kModeNoConfig = 0;
constexpr std::uint8_t kModeIdle = 1;
constexpr std::uint8_t kModeRunning = 2;
constexpr std::uint8_t kModeFault = 3;

struct Hello
{
  std::uint16_t protocol_version{0};
  std::uint8_t fw_major{0};
  std::uint8_t fw_minor{0};
  std::uint8_t wheels{0};
  std::uint32_t default_pulses_per_rev{0};
  std::uint32_t default_counts_per_rev{0};
};

struct Config
{
  std::uint32_t pulses_per_rev{1600};
  std::uint32_t counts_per_rev{4000};
  std::array<std::uint8_t, kWheels> motor_for_wheel{0, 1, 2, 3};  // FL, FR, RL, RR
  std::uint8_t dir_invert{0};  // bit m: motor m
  std::uint8_t enc_invert{0};  // bit m: motor m
  std::uint16_t watchdog_ms{200};
  float accel_backstop{5.0f};   // rad/s^2 at the wheel
  float max_wheel_speed{3.0f};  // rad/s
  std::int32_t following_error_limit{0};  // counts, 0 = off
};

struct ConfigAck
{
  std::uint8_t status{0};  // 0 ok, 1 invalid, 2 rejected while moving
};

struct Cmd
{
  std::uint16_t seq{0};
  std::uint8_t flags{0};
  std::array<float, kWheels> wheel_velocity{0.0f, 0.0f, 0.0f, 0.0f};  // rad/s, FL FR RL RR
};

struct State
{
  std::uint16_t seq{0};
  std::uint16_t last_cmd_seq{0};
  std::uint64_t mcu_time_us{0};
  std::array<std::int64_t, kWheels> counts{};
  std::array<std::int64_t, kWheels> pulses{};
  std::array<std::int32_t, kWheels> following_error{};
  std::uint16_t faults{0};
  std::uint16_t crc_errors{0};
  std::uint8_t mode{0};
};

/// Body sizes in bytes, without the type byte and the CRC.
constexpr std::size_t kHelloSize = 13;
constexpr std::size_t kConfigSize = 28;
constexpr std::size_t kConfigAckSize = 1;
constexpr std::size_t kCmdSize = 19;
constexpr std::size_t kStateSize = 97;

/// One decoded message; only the member of `type` is meaningful.
struct Message
{
  MsgType type{MsgType::kHello};
  Hello hello;
  Config config;
  ConfigAck config_ack;
  Cmd cmd;
  State state;
};

std::uint16_t crc16(const std::uint8_t * data, std::size_t len);
std::vector<std::uint8_t> cobs_encode(const std::vector<std::uint8_t> & data);
/// Empty optional for a malformed frame.
std::optional<std::vector<std::uint8_t>> cobs_decode(const std::uint8_t * data, std::size_t len);

/// Whole frames, with the trailing 0x00.
std::vector<std::uint8_t> encode_hello_req();
std::vector<std::uint8_t> encode_hello(const Hello & hello);
std::vector<std::uint8_t> encode_config(const Config & config);
std::vector<std::uint8_t> encode_config_ack(const ConfigAck & ack);
std::vector<std::uint8_t> encode_cmd(const Cmd & cmd);
std::vector<std::uint8_t> encode_state(const State & state);

/// Decode one frame given without its 0x00 delimiter. Empty for a bad COBS block, CRC, type or
/// size.
std::optional<Message> decode_frame(const std::uint8_t * frame, std::size_t len);

/// Splits a byte stream on 0x00 and decodes each frame; counts the bad ones.
class FrameReader
{
public:
  void feed(const std::uint8_t * data, std::size_t len, std::vector<Message> & out);
  std::size_t errors() const {return errors_;}

private:
  std::vector<std::uint8_t> buffer_;
  std::size_t errors_{0};
};

}  // namespace mecanum_hardware::drive_protocol

#endif  // MECANUM_HARDWARE__DRIVE_PROTOCOL_HPP_
