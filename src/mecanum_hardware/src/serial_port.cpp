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

#include "mecanum_hardware/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace mecanum_hardware
{

namespace
{

speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    case 4000000: return B4000000;
    default: return 0;
  }
}

}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baud)
{
  close();
  const speed_t speed = to_speed(baud);
  if (speed == 0) {
    error_ = "unsupported baud rate " + std::to_string(baud);
    return false;
  }
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd_ < 0) {
    error_ = device + ": " + std::strerror(errno);
    return false;
  }
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    error_ = device + ": tcgetattr: " + std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB);
  // Keep DTR/RTS as they are after opening: toggling them resets the DevKitC.
  tty.c_cflag &= ~HUPCL;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    error_ = device + ": tcsetattr: " + std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write(const std::uint8_t * data, std::size_t len)
{
  std::size_t done = 0;
  while (done < len) {
    const ssize_t n = ::write(fd_, data + done, len - done);
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }
      error_ = std::string("write: ") + std::strerror(errno);
      return false;
    }
    done += static_cast<std::size_t>(n);
  }
  return true;
}

int SerialPort::read(std::uint8_t * buffer, std::size_t len, int timeout_ms)
{
  if (fd_ < 0) {
    return -1;
  }
  pollfd pfd{fd_, POLLIN, 0};
  const int ready = ::poll(&pfd, 1, timeout_ms);
  if (ready < 0) {
    if (errno == EINTR) {
      return 0;
    }
    error_ = std::string("poll: ") + std::strerror(errno);
    return -1;
  }
  if (ready == 0) {
    return 0;
  }
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    error_ = "the port closed (device unplugged?)";
    return -1;
  }
  const ssize_t n = ::read(fd_, buffer, len);
  if (n < 0) {
    if (errno == EINTR || errno == EAGAIN) {
      return 0;
    }
    error_ = std::string("read: ") + std::strerror(errno);
    return -1;
  }
  return static_cast<int>(n);
}

}  // namespace mecanum_hardware
