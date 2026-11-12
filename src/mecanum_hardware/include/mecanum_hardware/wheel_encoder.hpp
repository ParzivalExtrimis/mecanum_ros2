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

#ifndef MECANUM_HARDWARE__WHEEL_ENCODER_HPP_
#define MECANUM_HARDWARE__WHEEL_ENCODER_HPP_

#include <cstdint>
#include <deque>

namespace mecanum_hardware
{

/// Settings of one wheel's encoder channel (decision 0006).
struct WheelEncoderConfig
{
  /// Quadrature counts per wheel revolution, after the decoding (x4) and any gearing.
  std::int64_t counts_per_rev{4000};
  /// Read cycles between a count being latched and it reaching ros2_control.
  int report_delay_cycles{0};
  /// Fault when the commanded wheel position and the counted one differ by more (rad).
  /// Zero or negative turns the check off.
  double following_error_limit{0.0};
};

/// The wheel state from a tapped quadrature encoder, as the hardware plugins report it.
///
/// It holds the rules of decision 0006, shared by the simulation plugin and the future ESP32
/// plugin:
/// - the position is a continuous 64-bit count, converted to radians;
/// - the velocity is the count change since the previous update divided by that update's
///   period, so a controller that integrates velocity x period reproduces the count exactly;
/// - the count may reach ros2_control a fixed number of cycles late (the MCU report and the
///   link);
/// - the commanded position (the integrated velocity command, the step pulses the MCU emits)
///   is compared with the undelayed count. Past the limit the encoder latches a fault, as the
///   MCU stops all wheels.
///
/// No ROS types, so it is unit tested on its own.
class WheelEncoder
{
public:
  explicit WheelEncoder(const WheelEncoderConfig & config);

  /// The count a shaft at `position` (rad) shows: floor, as a real counter.
  static std::int64_t quantize(double position, std::int64_t counts_per_rev);

  /// A new count, latched `period` seconds after the previous one.
  void update(std::int64_t count, double period);

  /// Adds the velocity command that the drive followed for `period` seconds; call it before
  /// the update() that closes that period. A
  /// non-finite command (no controller) stops the check until the next finite one: the
  /// commanded position then follows the count, so a wheel turned by hand does not fault.
  void command(double velocity, double period);

  /// Reported position (rad) and velocity (rad/s): delayed by report_delay_cycles.
  double position() const {return reported_position_;}
  double velocity() const {return reported_velocity_;}

  /// The newest, undelayed count.
  std::int64_t count() const {return count_;}
  /// Commanded minus counted position (rad), from the newest count.
  double following_error() const;
  /// Largest |following_error()| seen so far (rad).
  double max_following_error() const {return max_following_error_;}
  /// True once the following error passed the limit; stays set.
  bool fault() const {return fault_;}

  double radians_per_count() const {return radians_per_count_;}

private:
  struct Sample
  {
    std::int64_t count;
    double period;
  };

  void check_following_error();

  WheelEncoderConfig config_;
  double radians_per_count_;
  bool started_{false};
  bool commanding_{false};
  std::int64_t count_{0};
  // Oldest first: the reported sample is the front, report_delay_cycles + 2 entries at most.
  std::deque<Sample> samples_;
  double reported_position_{0.0};
  double reported_velocity_{0.0};
  double commanded_position_{0.0};
  double max_following_error_{0.0};
  bool fault_{false};
};

}  // namespace mecanum_hardware

#endif  // MECANUM_HARDWARE__WHEEL_ENCODER_HPP_
