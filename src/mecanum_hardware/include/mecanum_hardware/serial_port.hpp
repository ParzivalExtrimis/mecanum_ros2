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

#ifndef MECANUM_HARDWARE__SERIAL_PORT_HPP_
#define MECANUM_HARDWARE__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace mecanum_hardware
{

/// A raw POSIX serial port (8N1, no flow control), for the drive ESP32's USB-UART bridge.
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// Opens `device` at `baud` (one of the standard rates up to 4000000). Returns false and sets
  /// error() on failure.
  bool open(const std::string & device, int baud);
  void close();
  bool is_open() const {return fd_ >= 0;}

  /// Writes all of `data`; false on error (the link is then considered lost).
  bool write(const std::uint8_t * data, std::size_t len);
  /// Waits up to `timeout_ms` for data and reads what is there. Returns the byte count, 0 on
  /// timeout, -1 on error or a closed port.
  int read(std::uint8_t * buffer, std::size_t len, int timeout_ms);

  const std::string & error() const {return error_;}

private:
  int fd_{-1};
  std::string error_;
};

}  // namespace mecanum_hardware

#endif  // MECANUM_HARDWARE__SERIAL_PORT_HPP_
