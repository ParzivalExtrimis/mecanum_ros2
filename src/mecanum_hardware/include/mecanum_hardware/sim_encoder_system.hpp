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

#ifndef MECANUM_HARDWARE__SIM_ENCODER_SYSTEM_HPP_
#define MECANUM_HARDWARE__SIM_ENCODER_SYSTEM_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "mecanum_hardware/wheel_encoder.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace mecanum_hardware
{

/// Gazebo's GazeboSimSystem with the wheel state of the tapped encoders (decision 0006).
///
/// It wraps gz_ros2_control's GazeboSimSystem, held and loaded through pluginlib rather than
/// inherited: that class's private data type is not installed, so it cannot be built or
/// destroyed here. The inner system moves the wheel joints and reads their exact position from
/// physics. This class keeps the inner state handles to itself and exports its own position
/// and velocity for each joint, made by WheelEncoder from the exact position: a quantized count,
/// the velocity from count differences, and a report delay. It also integrates the velocity
/// commands into a commanded position, as the MCU counts the step pulses it emits, and on a
/// following error past the limit it stops the wheels and returns ERROR, as the ESP32 plugin
/// will on a fault.
///
/// Hardware parameters (ros2_control <hardware><param>):
///   counts_per_wheel_rev   quadrature counts per wheel revolution (default 4000)
///   report_delay_cycles    read cycles of delay (default 0)
///   following_error_limit  rad at the wheel, 0 = off (default 0)
///
/// Topics (shared with the ESP32 plugin):
///   /drive/inject_wheel_fault  std_msgs/String, a joint name: that wheel's count freezes,
///       as a dead encoder channel would, so the following error grows. Tests only.
///   /drive/following_error     std_msgs/Float64MultiArray at 1 Hz of sim time: per joint
///       the current following error, then per joint the largest |error| so far (rad).
class SimEncoderSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  SimEncoderSystem();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & system_info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, gz::sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    gz::sim::EntityComponentManager & ecm,
    unsigned int update_rate) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Wheel
  {
    std::string joint;
    std::unique_ptr<WheelEncoder> encoder;
    // The base class's handles, never exported.
    std::unique_ptr<hardware_interface::Handle> true_position;
    std::unique_ptr<hardware_interface::Handle> velocity_command;
    std::atomic<bool> frozen{false};
    bool frozen_latched{false};
    std::int64_t frozen_count{0};
  };

  void stop_wheels(const rclcpp::Time & time, const rclcpp::Duration & period);

  // Loaded through pluginlib, so it is built and destroyed inside gz_ros2_control's library.
  // The loader must outlive the instance: declared first, destroyed last.
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> loader_;
  pluginlib::UniquePtr<gz_ros2_control::GazeboSimSystemInterface> inner_;
  WheelEncoderConfig config_;
  // Sized once in on_init: the exported handles point into these vectors.
  std::vector<std::unique_ptr<Wheel>> wheels_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  bool faulted_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub_;
  double since_report_{0.0};
};

}  // namespace mecanum_hardware

#endif  // MECANUM_HARDWARE__SIM_ENCODER_SYSTEM_HPP_
