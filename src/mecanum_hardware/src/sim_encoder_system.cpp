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

#include "mecanum_hardware/sim_encoder_system.hpp"

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mecanum_hardware
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("mecanum_hardware.SimEncoderSystem");

std::string param(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}

}  // namespace

SimEncoderSystem::SimEncoderSystem()
: loader_("gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface"),
  inner_(loader_.createUniqueInstance("gz_ros2_control/GazeboSimSystem"))
{
}

bool SimEncoderSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, gz::sim::Entity> & joints,
  const hardware_interface::HardwareInfo & hardware_info,
  gz::sim::EntityComponentManager & ecm,
  unsigned int update_rate)
{
  // gz_ros2_control calls this before on_init.
  node_ = model_nh;
  return inner_->initSim(model_nh, joints, hardware_info, ecm, update_rate);
}

hardware_interface::CallbackReturn SimEncoderSystem::on_init(
  const hardware_interface::HardwareInfo & system_info)
{
  // The inner system needs the logger and clock that the framework gave this one.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto result = inner_->init(system_info, get_logger(), get_clock());
#pragma GCC diagnostic pop
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  try {
    config_.counts_per_rev = std::stoll(param(system_info, "counts_per_wheel_rev", "4000"));
    config_.report_delay_cycles = std::stoi(param(system_info, "report_delay_cycles", "0"));
    config_.following_error_limit = std::stod(param(system_info, "following_error_limit", "0"));
    WheelEncoder check(config_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(kLogger, "Bad encoder parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : system_info.joints) {
    auto wheel = std::make_unique<Wheel>();
    wheel->joint = joint.name;
    wheel->encoder = std::make_unique<WheelEncoder>(config_);
    wheels_.push_back(std::move(wheel));
  }
  positions_.assign(wheels_.size(), 0.0);
  velocities_.assign(wheels_.size(), 0.0);

  if (node_) {
    fault_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/drive/inject_wheel_fault", 10,
      [this](const std_msgs::msg::String & msg) {
        for (auto & wheel : wheels_) {
          if (wheel->joint == msg.data) {
            RCLCPP_WARN(kLogger, "Test: the encoder of %s stops counting", msg.data.c_str());
            wheel->frozen = true;
            return;
          }
        }
        RCLCPP_WARN(kLogger, "/drive/inject_wheel_fault: no joint named '%s'", msg.data.c_str());
      });
    error_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/drive/following_error", 10);
  }

  RCLCPP_INFO(
    kLogger, "Tapped encoders: %ld counts per wheel revolution, report delay %d cycles, "
    "following error limit %.3f rad", static_cast<long>(config_.counts_per_rev),  // NOLINT
    config_.report_delay_cycles, config_.following_error_limit);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// GazeboSimSystem still exports the old way (handles on its own doubles), so this does too.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
std::vector<hardware_interface::StateInterface> SimEncoderSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> exported;
  for (auto & base : inner_->export_state_interfaces()) {
    bool replaced = false;
    for (std::size_t i = 0; i < wheels_.size(); ++i) {
      if (base.get_prefix_name() != wheels_[i]->joint) {
        continue;
      }
      if (base.get_interface_name() == hardware_interface::HW_IF_POSITION) {
        wheels_[i]->true_position = std::make_unique<hardware_interface::Handle>(base);
        exported.emplace_back(wheels_[i]->joint, hardware_interface::HW_IF_POSITION,
          &positions_[i]);
        replaced = true;
      } else if (base.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
        exported.emplace_back(wheels_[i]->joint, hardware_interface::HW_IF_VELOCITY,
          &velocities_[i]);
        replaced = true;
      }
    }
    if (!replaced) {
      exported.push_back(base);
    }
  }
  return exported;
}

std::vector<hardware_interface::CommandInterface> SimEncoderSystem::export_command_interfaces()
{
  auto exported = inner_->export_command_interfaces();
  for (auto & command : exported) {
    for (auto & wheel : wheels_) {
      if (command.get_prefix_name() == wheel->joint &&
        command.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
      {
        wheel->velocity_command = std::make_unique<hardware_interface::Handle>(
          static_cast<const hardware_interface::Handle &>(command));
      }
    }
  }
  return exported;
}
#pragma GCC diagnostic pop

hardware_interface::CallbackReturn SimEncoderSystem::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  return inner_->on_configure(previous_state);
}

hardware_interface::CallbackReturn SimEncoderSystem::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return inner_->on_activate(previous_state);
}

hardware_interface::CallbackReturn SimEncoderSystem::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return inner_->on_deactivate(previous_state);
}

hardware_interface::return_type SimEncoderSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return inner_->perform_command_mode_switch(start_interfaces, stop_interfaces);
}

hardware_interface::return_type SimEncoderSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto result = inner_->read(time, period);
  if (result != hardware_interface::return_type::OK) {
    return result;
  }
  const double dt = period.seconds();
  for (std::size_t i = 0; i < wheels_.size(); ++i) {
    Wheel & wheel = *wheels_[i];
    if (!wheel.true_position) {
      continue;
    }
    // The command in the handle is the one the controller wrote at the previous update; the
    // drive followed it for the period that just ended. (gz_ros2_control calls write() on every
    // physics step but read() once per control period, so the integration happens here.)
    if (wheel.velocity_command) {
      wheel.encoder->command(
        wheel.velocity_command->get_optional().value_or(std::nan("")), dt);
    }
    const double exact = wheel.true_position->get_optional().value_or(0.0);
    std::int64_t count = WheelEncoder::quantize(exact, config_.counts_per_rev);
    if (wheel.frozen) {
      if (!wheel.frozen_latched) {
        wheel.frozen_latched = true;
        wheel.frozen_count = wheel.encoder->count();
      }
      count = wheel.frozen_count;
    }
    wheel.encoder->update(count, dt);
    positions_[i] = wheel.encoder->position();
    velocities_[i] = wheel.encoder->velocity();
  }

  since_report_ += dt;
  if (error_pub_ && since_report_ >= 1.0) {
    since_report_ = 0.0;
    std_msgs::msg::Float64MultiArray msg;
    for (const auto & wheel : wheels_) {
      msg.data.push_back(wheel->encoder->following_error());
    }
    for (const auto & wheel : wheels_) {
      msg.data.push_back(wheel->encoder->max_following_error());
    }
    error_pub_->publish(msg);
  }

  for (const auto & wheel : wheels_) {
    if (wheel->encoder->fault()) {
      if (!faulted_) {
        faulted_ = true;
        RCLCPP_ERROR(
          kLogger, "Following error on %s: %.3f rad (limit %.3f). Stopping all wheels.",
          wheel->joint.c_str(), wheel->encoder->following_error(),
          config_.following_error_limit);
        stop_wheels(time, period);
      }
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SimEncoderSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (faulted_) {
    stop_wheels(time, period);
    return hardware_interface::return_type::ERROR;
  }
  return inner_->write(time, period);
}

void SimEncoderSystem::stop_wheels(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // As the MCU on a fault: every wheel to zero, whatever the controller commands.
  for (auto & wheel : wheels_) {
    if (wheel->velocity_command) {
      (void)wheel->velocity_command->set_value(0.0);
    }
  }
  (void)inner_->write(time, period);
}

}  // namespace mecanum_hardware

PLUGINLIB_EXPORT_CLASS(
  mecanum_hardware::SimEncoderSystem, gz_ros2_control::GazeboSimSystemInterface)
