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

#ifndef MECANUM_HARDWARE__ESP32_SYSTEM_HPP_
#define MECANUM_HARDWARE__ESP32_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "mecanum_hardware/drive_protocol.hpp"
#include "mecanum_hardware/serial_port.hpp"
#include "mecanum_hardware/wheel_encoder.hpp"
#include "mecanum_interfaces/msg/drive_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mecanum_hardware
{

/// The real drive: the ESP32 firmware (firmware/mecanum_drive_esp32) over USB serial.
///
/// Each wheel joint has a velocity command and position and velocity states. The states come
/// from the tapped encoder counts in the ESP32's STATE reports, through WheelEncoder (decision
/// 0006): position from the count, velocity from the count change over the read period. The
/// ESP32 checks the following error itself (emitted pulses against counts) and latches a fault;
/// a latched fault, a stale report or a lost link makes read() return ERROR, so ros2_control
/// deactivates the controllers and /joint_states stops.
///
/// Hardware parameters (ros2_control <hardware><param>), see the esp32 branch of
/// mecanum_controllers/urdf/mecanum.ros2_control.xacro:
///   port, baud                      the serial device (/dev/serial/by-id/...) and 921600
///   counts_per_wheel_rev            encoder counts per wheel revolution (4000)
///   pulses_per_rev                  driver pulses per revolution, the CS-D808 DIP (1600)
///   motor_for_wheel                 motor index 0..3 for FL FR RL RR, e.g. "0 1 2 3"
///   dir_invert, enc_invert          per motor 0/1, e.g. "0 0 1 1"
///   watchdog_ms                     the ESP32 stops without a command for this long
///   accel_backstop                  rad/s^2 at the wheel, the ESP32's ramp
///   max_wheel_speed                 rad/s, the ESP32 clamps above it
///   following_error_limit           rad at the wheel, 0 = off
///   state_timeout_ms                read() fails when the newest report is older
///   connect_timeout_s               time to wait for HELLO and CONFIG_ACK
///
/// Topics (the same names as the simulation's SimEncoderSystem where they exist):
///   /drive/state            mecanum_interfaces/DriveState, every report (100 Hz)
///   /drive/following_error  std_msgs/Float64MultiArray at 1 Hz: per wheel the current
///                           following error, then the largest |error| since activation (rad)
class Esp32System : public hardware_interface::SystemInterface
{
public:
  ~Esp32System() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;

  void reader_loop();
  void handle(const drive_protocol::Message & message, Clock::time_point received);
  bool send(const std::vector<std::uint8_t> & frame);
  void send_stop();
  void stop_reader();
  bool wait_for(const std::function<bool()> & done, double seconds);

  // Parameters.
  std::string port_;
  int baud_{921600};
  drive_protocol::Config config_;
  double following_error_limit_rad_{0.0};
  double state_timeout_s_{0.1};
  double connect_timeout_s_{3.0};

  // Joint order in info_.joints -> wheel index (FL, FR, RL, RR).
  std::array<std::size_t, drive_protocol::kWheels> joint_of_wheel_{};
  std::array<std::string, drive_protocol::kWheels> wheel_joint_name_{};
  std::array<double, drive_protocol::kWheels> positions_{};
  std::array<double, drive_protocol::kWheels> velocities_{};
  std::array<double, drive_protocol::kWheels> commands_{};
  std::vector<std::unique_ptr<WheelEncoder>> encoders_;

  SerialPort serial_;
  std::mutex write_mutex_;
  std::thread reader_;
  std::atomic<bool> running_{false};
  std::atomic<bool> link_lost_{false};
  std::atomic<bool> active_{false};
  std::uint16_t cmd_seq_{0};

  // Shared with the reader thread.
  std::mutex mutex_;
  std::optional<drive_protocol::State> state_;
  Clock::time_point state_time_{};
  std::optional<drive_protocol::Hello> hello_;
  int acks_{0};
  std::uint8_t last_ack_status_{0};
  std::size_t host_rx_errors_{0};
  std::array<float, drive_protocol::kWheels> sent_command_{};
  std::array<double, drive_protocol::kWheels> max_following_error_{};
  Clock::time_point last_error_report_{};
  bool fault_reported_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<mecanum_interfaces::msg::DriveState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub_;
};

}  // namespace mecanum_hardware

#endif  // MECANUM_HARDWARE__ESP32_SYSTEM_HPP_
