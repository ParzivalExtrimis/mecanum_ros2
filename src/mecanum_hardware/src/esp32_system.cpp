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

#include "mecanum_hardware/esp32_system.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mecanum_hardware
{

namespace dp = drive_protocol;

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("mecanum_hardware.Esp32System");
constexpr double kTwoPi = 6.283185307179586;
// Joint name fragments, in the protocol's wheel order.
const std::array<const char *, dp::kWheels> kWheelNames = {
  "front_left", "front_right", "rear_left", "rear_right"};

std::string param(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}

std::vector<int> int_list(const std::string & text)
{
  std::vector<int> values;
  std::string item;
  for (char c : text) {
    if (c == ',' || c == ' ' || c == '\t') {
      if (!item.empty()) {
        values.push_back(std::stoi(item));
        item.clear();
      }
    } else {
      item.push_back(c);
    }
  }
  if (!item.empty()) {
    values.push_back(std::stoi(item));
  }
  return values;
}

std::uint8_t mask_of(const std::vector<int> & bits, const std::string & name)
{
  if (bits.size() != dp::kWheels) {
    throw std::invalid_argument(name + " needs 4 values (motor 1..4), e.g. \"0 0 0 0\"");
  }
  std::uint8_t mask = 0;
  for (std::size_t m = 0; m < bits.size(); ++m) {
    if (bits[m] != 0) {
      mask |= static_cast<std::uint8_t>(1u << m);
    }
  }
  return mask;
}

std::string describe_faults(std::uint16_t faults)
{
  std::ostringstream text;
  for (int w = 0; w < dp::kWheels; ++w) {
    if (faults & (dp::kFaultFollowingErrorWheel0 << w)) {
      text << "following error " << kWheelNames[w] << "; ";
    }
  }
  if (faults & dp::kFaultWatchdog) {text << "watchdog (no command); ";}
  if (faults & dp::kFaultNoConfig) {text << "no configuration; ";}
  if (faults & dp::kFaultOverspeed) {text << "speed clamped; ";}
  if (faults & dp::kFaultStepDriver) {text << "step generator or encoders failed to start; ";}
  return text.str();
}

}  // namespace

Esp32System::~Esp32System()
{
  stop_reader();
}

hardware_interface::CallbackReturn Esp32System::on_init(
  const hardware_interface::HardwareInfo & info)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // Jazzy's framework still calls this overload; the base stores info_.
  const auto base = SystemInterface::on_init(info);
#pragma GCC diagnostic pop
  if (base != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    port_ = param(info, "port", "");
    if (port_.empty()) {
      throw std::invalid_argument("the 'port' parameter is required");
    }
    baud_ = std::stoi(param(info, "baud", "921600"));
    config_.counts_per_rev = static_cast<std::uint32_t>(
      std::stoul(param(info, "counts_per_wheel_rev", "4000")));
    config_.pulses_per_rev = static_cast<std::uint32_t>(
      std::stoul(param(info, "pulses_per_rev", "1600")));
    const auto order = int_list(param(info, "motor_for_wheel", "0 1 2 3"));
    if (order.size() != dp::kWheels) {
      throw std::invalid_argument("motor_for_wheel needs 4 motor indices (FL FR RL RR)");
    }
    std::array<bool, dp::kWheels> seen{};
    for (int w = 0; w < dp::kWheels; ++w) {
      if (order[w] < 0 || order[w] > 3 || seen[order[w]]) {
        throw std::invalid_argument("motor_for_wheel must be a permutation of 0 1 2 3");
      }
      seen[order[w]] = true;
      config_.motor_for_wheel[w] = static_cast<std::uint8_t>(order[w]);
    }
    config_.dir_invert = mask_of(int_list(param(info, "dir_invert", "0 0 0 0")), "dir_invert");
    config_.enc_invert = mask_of(int_list(param(info, "enc_invert", "0 0 0 0")), "enc_invert");
    config_.watchdog_ms = static_cast<std::uint16_t>(std::stoi(param(info, "watchdog_ms", "200")));
    config_.accel_backstop = std::stof(param(info, "accel_backstop", "5.0"));
    config_.max_wheel_speed = std::stof(param(info, "max_wheel_speed", "3.0"));
    following_error_limit_rad_ = std::stod(param(info, "following_error_limit", "0.5"));
    config_.following_error_limit = static_cast<std::int32_t>(std::lround(
      following_error_limit_rad_ * config_.counts_per_rev / kTwoPi));
    state_timeout_s_ = std::stod(param(info, "state_timeout_ms", "100")) / 1000.0;
    connect_timeout_s_ = std::stod(param(info, "connect_timeout_s", "3.0"));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(kLogger, "Bad hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.joints.size() != dp::kWheels) {
    RCLCPP_FATAL(kLogger, "Expected 4 wheel joints, got %zu", info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::array<bool, dp::kWheels> found{};
  for (std::size_t j = 0; j < info.joints.size(); ++j) {
    const auto & joint = info.joints[j];
    int wheel = -1;
    for (int w = 0; w < dp::kWheels; ++w) {
      if (joint.name.find(kWheelNames[w]) != std::string::npos) {
        wheel = w;
      }
    }
    if (wheel < 0 || found[wheel]) {
      RCLCPP_FATAL(kLogger, "Joint %s is not a distinct front/rear left/right wheel",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(kLogger, "Joint %s needs one velocity command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    found[wheel] = true;
    joint_of_wheel_[wheel] = j;
    wheel_joint_name_[wheel] = joint.name;
  }

  WheelEncoderConfig encoder;
  encoder.counts_per_rev = config_.counts_per_rev;
  encoder.report_delay_cycles = 0;     // the reports are real; nothing to add
  encoder.following_error_limit = 0.0;  // the ESP32 checks it against its real pulses
  encoders_.clear();
  for (int w = 0; w < dp::kWheels; ++w) {
    encoders_.push_back(std::make_unique<WheelEncoder>(encoder));
  }

  node_ = rclcpp::Node::make_shared("esp32_drive");
  state_pub_ = node_->create_publisher<mecanum_interfaces::msg::DriveState>("/drive/state", 50);
  error_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/drive/following_error", 10);

  RCLCPP_INFO(
    kLogger, "ESP32 drive on %s at %d baud: %u counts and %u pulses per wheel revolution, "
    "following error limit %.3f rad (%d counts)", port_.c_str(), baud_,
    config_.counts_per_rev, config_.pulses_per_rev, following_error_limit_rad_,
    config_.following_error_limit);
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool Esp32System::send(const std::vector<std::uint8_t> & frame)
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (!serial_.is_open()) {
    return false;
  }
  if (!serial_.write(frame.data(), frame.size())) {
    link_lost_ = true;
    return false;
  }
  return true;
}

void Esp32System::send_stop()
{
  dp::Cmd cmd;
  cmd.seq = ++cmd_seq_;
  cmd.flags = 0;  // motion off: every wheel ramps to zero
  send(dp::encode_cmd(cmd));
}

bool Esp32System::wait_for(const std::function<bool()> & done, double seconds)
{
  const auto end = Clock::now() + std::chrono::duration<double>(seconds);
  while (Clock::now() < end) {
    if (done()) {
      return true;
    }
    if (link_lost_) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return done();
}

hardware_interface::CallbackReturn Esp32System::on_configure(const rclcpp_lifecycle::State &)
{
  stop_reader();
  if (!serial_.open(port_, baud_)) {
    RCLCPP_ERROR(kLogger, "Cannot open the ESP32 port: %s", serial_.error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  link_lost_ = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hello_.reset();
    state_.reset();
  }
  running_ = true;
  reader_ = std::thread(&Esp32System::reader_loop, this);

  // Opening the port may reset the board; ask for HELLO until it answers.
  auto last_request = Clock::now() - std::chrono::seconds(1);
  const bool answered = wait_for(
    [this, &last_request]() {
      if (Clock::now() - last_request > std::chrono::milliseconds(250)) {
        last_request = Clock::now();
        send(dp::encode_hello_req());
      }
      std::lock_guard<std::mutex> lock(mutex_);
      return hello_.has_value();
    }, connect_timeout_s_);
  if (!answered) {
    RCLCPP_ERROR(
      kLogger, "No HELLO from the ESP32 on %s within %.1f s. Is the drive sketch uploaded, "
      "the serial monitor closed, and the port right?", port_.c_str(), connect_timeout_s_);
    stop_reader();
    return hardware_interface::CallbackReturn::ERROR;
  }
  dp::Hello hello;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hello = *hello_;
  }
  if (hello.protocol_version != dp::kProtocolVersion) {
    RCLCPP_ERROR(
      kLogger, "The ESP32 speaks protocol %u, this plugin %u: upload the matching sketch",
      hello.protocol_version, dp::kProtocolVersion);
    stop_reader();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    kLogger, "ESP32 drive firmware %u.%u, protocol %u, defaults %u pulses / %u counts per rev",
    hello.fw_major, hello.fw_minor, hello.protocol_version, hello.default_pulses_per_rev,
    hello.default_counts_per_rev);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32System::on_activate(const rclcpp_lifecycle::State &)
{
  // Configure; the ESP32 accepts it only with every wheel still.
  int acks_before = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    acks_before = acks_;
  }
  bool configured = false;
  for (int attempt = 0; attempt < 3 && !configured; ++attempt) {
    send(dp::encode_config(config_));
    const bool acked = wait_for(
      [this, acks_before]() {
        std::lock_guard<std::mutex> lock(mutex_);
        return acks_ > acks_before;
      }, 1.0);
    std::uint8_t status = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status = last_ack_status_;
      acks_before = acks_;
    }
    if (acked && status == 0) {
      configured = true;
    } else if (acked) {
      RCLCPP_WARN(kLogger, "The ESP32 rejected the configuration (status %u: %s)", status,
        status == 1 ? "invalid values" : "wheels still moving");
      if (status == 1) {
        return hardware_interface::CallbackReturn::ERROR;
      }
      send_stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
  if (!configured) {
    RCLCPP_ERROR(kLogger, "No CONFIG_ACK from the ESP32");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Clear any latched fault (accepted only when all wheels are still), motion still off.
  const bool cleared = wait_for(
    [this]() {
      dp::Cmd cmd;
      cmd.seq = ++cmd_seq_;
      cmd.flags = dp::kFlagClearFault;
      send(dp::encode_cmd(cmd));
      std::lock_guard<std::mutex> lock(mutex_);
      return state_ && state_->mode == dp::kModeIdle && !(state_->faults & dp::kFaultLatched);
    }, 2.0);
  if (!cleared) {
    std::uint16_t faults = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      faults = state_ ? state_->faults : 0;
    }
    RCLCPP_ERROR(kLogger, "The ESP32 does not come out of its fault: %s",
      describe_faults(faults).c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  WheelEncoderConfig encoder;
  encoder.counts_per_rev = config_.counts_per_rev;
  for (auto & e : encoders_) {
    e = std::make_unique<WheelEncoder>(encoder);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int w = 0; w < dp::kWheels; ++w) {
      encoders_[w]->update(state_->counts[w], 0.0);
      positions_[w] = encoders_[w]->position();
      velocities_[w] = 0.0;
      commands_[w] = 0.0;
      max_following_error_[w] = 0.0;
    }
    fault_reported_ = false;
  }
  active_ = true;
  RCLCPP_INFO(kLogger, "ESP32 drive active");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32System::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  for (int i = 0; i < 3; ++i) {
    send_stop();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32System::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;
  send_stop();
  stop_reader();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32System::on_shutdown(const rclcpp_lifecycle::State &)
{
  return on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn Esp32System::on_error(const rclcpp_lifecycle::State &)
{
  // Back to unconfigured: stop the wheels and release the port, so a later configure starts
  // clean (ros2 control set_hardware_component_state mecanum_base active).
  active_ = false;
  send_stop();
  stop_reader();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void Esp32System::stop_reader()
{
  running_ = false;
  if (reader_.joinable()) {
    reader_.join();
  }
  std::lock_guard<std::mutex> lock(write_mutex_);
  serial_.close();
}

std::vector<hardware_interface::StateInterface> Esp32System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for (int w = 0; w < dp::kWheels; ++w) {
    interfaces.emplace_back(wheel_joint_name_[w], hardware_interface::HW_IF_POSITION,
      &positions_[w]);
    interfaces.emplace_back(wheel_joint_name_[w], hardware_interface::HW_IF_VELOCITY,
      &velocities_[w]);
  }
#pragma GCC diagnostic pop
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> Esp32System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for (int w = 0; w < dp::kWheels; ++w) {
    interfaces.emplace_back(wheel_joint_name_[w], hardware_interface::HW_IF_VELOCITY,
      &commands_[w]);
  }
#pragma GCC diagnostic pop
  return interfaces;
}

hardware_interface::return_type Esp32System::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (link_lost_) {
    RCLCPP_ERROR(kLogger, "Lost the ESP32 link: %s", serial_.error().c_str());
    return hardware_interface::return_type::ERROR;
  }
  dp::State state;
  Clock::time_point received;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_) {
      RCLCPP_ERROR(kLogger, "No STATE from the ESP32");
      return hardware_interface::return_type::ERROR;
    }
    state = *state_;
    received = state_time_;
  }
  const double age = std::chrono::duration<double>(Clock::now() - received).count();
  if (age > state_timeout_s_) {
    RCLCPP_ERROR(kLogger, "The newest ESP32 STATE is %.0f ms old (limit %.0f ms)",
      age * 1000.0, state_timeout_s_ * 1000.0);
    return hardware_interface::return_type::ERROR;
  }
  if (state.faults & dp::kFaultLatched) {
    if (!fault_reported_) {
      fault_reported_ = true;
      std::ostringstream fe;
      for (int w = 0; w < dp::kWheels; ++w) {
        fe << " " << kWheelNames[w] << " " << state.following_error[w];
      }
      RCLCPP_ERROR(kLogger, "ESP32 fault, all wheels stopped: %sfollowing error (counts):%s",
        describe_faults(state.faults).c_str(), fe.str().c_str());
    }
    return hardware_interface::return_type::ERROR;
  }
  const double dt = period.seconds();
  for (int w = 0; w < dp::kWheels; ++w) {
    encoders_[w]->update(state.counts[w], dt);
    positions_[w] = encoders_[w]->position();
    velocities_[w] = encoders_[w]->velocity();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Esp32System::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  dp::Cmd cmd;
  cmd.seq = ++cmd_seq_;
  cmd.flags = active_ ? dp::kFlagEnable : 0;
  for (int w = 0; w < dp::kWheels; ++w) {
    const double v = commands_[w];
    cmd.wheel_velocity[w] = std::isfinite(v) ? static_cast<float>(v) : 0.0f;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sent_command_ = cmd.wheel_velocity;
  }
  if (!send(dp::encode_cmd(cmd))) {
    RCLCPP_ERROR(kLogger, "Cannot write to the ESP32: %s", serial_.error().c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void Esp32System::reader_loop()
{
  dp::FrameReader reader;
  std::vector<std::uint8_t> buffer(1024);
  std::vector<dp::Message> messages;
  while (running_) {
    const int n = serial_.read(buffer.data(), buffer.size(), 50);
    if (n < 0) {
      link_lost_ = true;
      RCLCPP_ERROR(kLogger, "ESP32 serial link: %s", serial_.error().c_str());
      return;
    }
    if (n == 0) {
      continue;
    }
    const auto received = Clock::now();
    messages.clear();
    reader.feed(buffer.data(), static_cast<std::size_t>(n), messages);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      host_rx_errors_ = reader.errors();
    }
    for (const auto & m : messages) {
      handle(m, received);
    }
  }
}

void Esp32System::handle(const dp::Message & m, Clock::time_point received)
{
  if (m.type == dp::MsgType::kHello) {
    std::lock_guard<std::mutex> lock(mutex_);
    hello_ = m.hello;
    return;
  }
  if (m.type == dp::MsgType::kConfigAck) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++acks_;
    last_ack_status_ = m.config_ack.status;
    return;
  }
  if (m.type != dp::MsgType::kState) {
    return;
  }
  const dp::State & s = m.state;
  mecanum_interfaces::msg::DriveState msg;
  bool report_errors = false;
  std_msgs::msg::Float64MultiArray errors;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = s;
    state_time_ = received;
    for (int w = 0; w < dp::kWheels; ++w) {
      const double fe = s.following_error[w] * kTwoPi / config_.counts_per_rev;
      max_following_error_[w] = std::max(max_following_error_[w], std::abs(fe));
      msg.command[w] = sent_command_[w];
    }
    msg.host_rx_errors = static_cast<std::uint32_t>(host_rx_errors_);
    if (received - last_error_report_ > std::chrono::seconds(1)) {
      last_error_report_ = received;
      report_errors = true;
      for (int w = 0; w < dp::kWheels; ++w) {
        errors.data.push_back(s.following_error[w] * kTwoPi / config_.counts_per_rev);
      }
      for (int w = 0; w < dp::kWheels; ++w) {
        errors.data.push_back(max_following_error_[w]);
      }
    }
  }
  msg.header.stamp = node_->get_clock()->now();
  msg.seq = s.seq;
  msg.mcu_time_us = s.mcu_time_us;
  for (int w = 0; w < dp::kWheels; ++w) {
    msg.counts[w] = s.counts[w];
    msg.pulses[w] = s.pulses[w];
    msg.following_error[w] = s.following_error[w];
  }
  msg.faults = s.faults;
  msg.mode = s.mode;
  msg.mcu_rx_errors = s.crc_errors;
  state_pub_->publish(msg);
  if (report_errors) {
    error_pub_->publish(errors);
  }
}

}  // namespace mecanum_hardware

PLUGINLIB_EXPORT_CLASS(mecanum_hardware::Esp32System, hardware_interface::SystemInterface)
