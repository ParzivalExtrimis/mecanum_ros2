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

#include "mecanum_hardware/drive_protocol.hpp"

#include <cstring>
#include <optional>
#include <vector>

namespace mecanum_hardware::drive_protocol
{

static_assert(
  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
  "the protocol is little-endian and is packed with memcpy");

namespace
{

class Writer
{
public:
  explicit Writer(MsgType type) {bytes_.push_back(static_cast<std::uint8_t>(type));}

  template<typename T>
  void put(T value)
  {
    std::uint8_t raw[sizeof(T)];
    std::memcpy(raw, &value, sizeof(T));
    bytes_.insert(bytes_.end(), raw, raw + sizeof(T));
  }

  std::vector<std::uint8_t> frame()
  {
    const std::uint16_t crc = crc16(bytes_.data(), bytes_.size());
    put(crc);
    auto out = cobs_encode(bytes_);
    out.push_back(0x00);
    return out;
  }

private:
  std::vector<std::uint8_t> bytes_;
};

class Reader
{
public:
  Reader(const std::uint8_t * data, std::size_t len)
  : data_(data), len_(len) {}

  template<typename T>
  T get()
  {
    T value{};
    if (pos_ + sizeof(T) <= len_) {
      std::memcpy(&value, data_ + pos_, sizeof(T));
    }
    pos_ += sizeof(T);
    return value;
  }

private:
  const std::uint8_t * data_;
  std::size_t len_;
  std::size_t pos_{0};
};

}  // namespace

std::uint16_t crc16(const std::uint8_t * data, std::size_t len)
{
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<std::uint8_t> cobs_encode(const std::vector<std::uint8_t> & data)
{
  std::vector<std::uint8_t> out;
  out.reserve(data.size() + data.size() / 254 + 2);
  std::size_t code_pos = 0;
  out.push_back(0);
  std::uint8_t code = 1;
  for (const std::uint8_t byte : data) {
    if (byte == 0) {
      out[code_pos] = code;
      code_pos = out.size();
      out.push_back(0);
      code = 1;
    } else {
      out.push_back(byte);
      ++code;
      if (code == 0xFF) {
        out[code_pos] = code;
        code_pos = out.size();
        out.push_back(0);
        code = 1;
      }
    }
  }
  out[code_pos] = code;
  return out;
}

std::optional<std::vector<std::uint8_t>> cobs_decode(const std::uint8_t * data, std::size_t len)
{
  std::vector<std::uint8_t> out;
  out.reserve(len);
  std::size_t i = 0;
  while (i < len) {
    const std::uint8_t code = data[i];
    if (code == 0 || i + code > len) {
      return std::nullopt;
    }
    out.insert(out.end(), data + i + 1, data + i + code);
    i += code;
    if (code < 0xFF && i < len) {
      out.push_back(0);
    }
  }
  return out;
}

std::vector<std::uint8_t> encode_hello_req()
{
  return Writer(MsgType::kHelloReq).frame();
}

std::vector<std::uint8_t> encode_hello(const Hello & h)
{
  Writer w(MsgType::kHello);
  w.put(h.protocol_version);
  w.put(h.fw_major);
  w.put(h.fw_minor);
  w.put(h.wheels);
  w.put(h.default_pulses_per_rev);
  w.put(h.default_counts_per_rev);
  return w.frame();
}

std::vector<std::uint8_t> encode_config(const Config & c)
{
  Writer w(MsgType::kConfig);
  w.put(c.pulses_per_rev);
  w.put(c.counts_per_rev);
  for (const auto m : c.motor_for_wheel) {
    w.put(m);
  }
  w.put(c.dir_invert);
  w.put(c.enc_invert);
  w.put(c.watchdog_ms);
  w.put(c.accel_backstop);
  w.put(c.max_wheel_speed);
  w.put(c.following_error_limit);
  return w.frame();
}

std::vector<std::uint8_t> encode_config_ack(const ConfigAck & a)
{
  Writer w(MsgType::kConfigAck);
  w.put(a.status);
  return w.frame();
}

std::vector<std::uint8_t> encode_cmd(const Cmd & c)
{
  Writer w(MsgType::kCmd);
  w.put(c.seq);
  w.put(c.flags);
  for (const auto v : c.wheel_velocity) {
    w.put(v);
  }
  return w.frame();
}

std::vector<std::uint8_t> encode_state(const State & s)
{
  Writer w(MsgType::kState);
  w.put(s.seq);
  w.put(s.last_cmd_seq);
  w.put(s.mcu_time_us);
  for (const auto v : s.counts) {
    w.put(v);
  }
  for (const auto v : s.pulses) {
    w.put(v);
  }
  for (const auto v : s.following_error) {
    w.put(v);
  }
  w.put(s.faults);
  w.put(s.crc_errors);
  w.put(s.mode);
  return w.frame();
}

std::optional<Message> decode_frame(const std::uint8_t * frame, std::size_t len)
{
  const auto payload = cobs_decode(frame, len);
  if (!payload || payload->size() < 3) {
    return std::nullopt;
  }
  const std::size_t n = payload->size();
  const std::uint16_t crc = static_cast<std::uint16_t>(
    (*payload)[n - 2] | ((*payload)[n - 1] << 8));
  if (crc16(payload->data(), n - 2) != crc) {
    return std::nullopt;
  }
  Message m;
  m.type = static_cast<MsgType>((*payload)[0]);
  const std::uint8_t * body = payload->data() + 1;
  const std::size_t size = n - 3;
  Reader r(body, size);
  switch (m.type) {
    case MsgType::kHello:
      if (size != kHelloSize) {return std::nullopt;}
      m.hello.protocol_version = r.get<std::uint16_t>();
      m.hello.fw_major = r.get<std::uint8_t>();
      m.hello.fw_minor = r.get<std::uint8_t>();
      m.hello.wheels = r.get<std::uint8_t>();
      m.hello.default_pulses_per_rev = r.get<std::uint32_t>();
      m.hello.default_counts_per_rev = r.get<std::uint32_t>();
      return m;
    case MsgType::kHelloReq:
      if (size != 0) {return std::nullopt;}
      return m;
    case MsgType::kConfig:
      if (size != kConfigSize) {return std::nullopt;}
      m.config.pulses_per_rev = r.get<std::uint32_t>();
      m.config.counts_per_rev = r.get<std::uint32_t>();
      for (auto & v : m.config.motor_for_wheel) {
        v = r.get<std::uint8_t>();
      }
      m.config.dir_invert = r.get<std::uint8_t>();
      m.config.enc_invert = r.get<std::uint8_t>();
      m.config.watchdog_ms = r.get<std::uint16_t>();
      m.config.accel_backstop = r.get<float>();
      m.config.max_wheel_speed = r.get<float>();
      m.config.following_error_limit = r.get<std::int32_t>();
      return m;
    case MsgType::kConfigAck:
      if (size != kConfigAckSize) {return std::nullopt;}
      m.config_ack.status = r.get<std::uint8_t>();
      return m;
    case MsgType::kCmd:
      if (size != kCmdSize) {return std::nullopt;}
      m.cmd.seq = r.get<std::uint16_t>();
      m.cmd.flags = r.get<std::uint8_t>();
      for (auto & v : m.cmd.wheel_velocity) {
        v = r.get<float>();
      }
      return m;
    case MsgType::kState:
      if (size != kStateSize) {return std::nullopt;}
      m.state.seq = r.get<std::uint16_t>();
      m.state.last_cmd_seq = r.get<std::uint16_t>();
      m.state.mcu_time_us = r.get<std::uint64_t>();
      for (auto & v : m.state.counts) {
        v = r.get<std::int64_t>();
      }
      for (auto & v : m.state.pulses) {
        v = r.get<std::int64_t>();
      }
      for (auto & v : m.state.following_error) {
        v = r.get<std::int32_t>();
      }
      m.state.faults = r.get<std::uint16_t>();
      m.state.crc_errors = r.get<std::uint16_t>();
      m.state.mode = r.get<std::uint8_t>();
      return m;
  }
  return std::nullopt;
}

void FrameReader::feed(const std::uint8_t * data, std::size_t len, std::vector<Message> & out)
{
  for (std::size_t i = 0; i < len; ++i) {
    if (data[i] != 0) {
      if (buffer_.size() < 512) {
        buffer_.push_back(data[i]);
      }
      continue;
    }
    if (!buffer_.empty()) {
      auto m = decode_frame(buffer_.data(), buffer_.size());
      if (m) {
        out.push_back(*m);
      } else {
        ++errors_;
      }
      buffer_.clear();
    }
  }
}

}  // namespace mecanum_hardware::drive_protocol
