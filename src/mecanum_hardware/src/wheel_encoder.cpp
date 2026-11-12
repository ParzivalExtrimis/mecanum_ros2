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

#include "mecanum_hardware/wheel_encoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace mecanum_hardware
{

namespace
{
constexpr double kTwoPi = 6.283185307179586;
}  // namespace

WheelEncoder::WheelEncoder(const WheelEncoderConfig & config)
: config_(config)
{
  if (config_.counts_per_rev <= 0) {
    throw std::invalid_argument("counts_per_rev must be positive");
  }
  if (config_.report_delay_cycles < 0) {
    throw std::invalid_argument("report_delay_cycles must not be negative");
  }
  radians_per_count_ = kTwoPi / static_cast<double>(config_.counts_per_rev);
}

std::int64_t WheelEncoder::quantize(double position, std::int64_t counts_per_rev)
{
  return static_cast<std::int64_t>(
    std::floor(position * static_cast<double>(counts_per_rev) / kTwoPi));
}

void WheelEncoder::update(std::int64_t count, double period)
{
  count_ = count;
  started_ = true;
  if (!commanding_) {
    // No drive command: the commanded position follows the shaft.
    commanded_position_ = static_cast<double>(count) * radians_per_count_;
  }
  samples_.push_back({count, period});
  const std::size_t keep = static_cast<std::size_t>(config_.report_delay_cycles) + 2;
  while (samples_.size() > keep) {
    samples_.pop_front();
  }

  // The reported sample is report_delay_cycles behind the newest; until that many exist, the
  // oldest one is held.
  const std::ptrdiff_t newest = static_cast<std::ptrdiff_t>(samples_.size()) - 1;
  const std::ptrdiff_t index = std::max<std::ptrdiff_t>(0, newest - config_.report_delay_cycles);
  const Sample & reported = samples_[static_cast<std::size_t>(index)];
  reported_position_ = static_cast<double>(reported.count) * radians_per_count_;
  if (index >= 1 && reported.period > 0.0) {
    const Sample & before = samples_[static_cast<std::size_t>(index - 1)];
    reported_velocity_ =
      static_cast<double>(reported.count - before.count) * radians_per_count_ / reported.period;
  } else {
    reported_velocity_ = 0.0;
  }

  check_following_error();
}

void WheelEncoder::command(double velocity, double period)
{
  if (!started_ || !std::isfinite(velocity)) {
    commanding_ = false;
    commanded_position_ = static_cast<double>(count_) * radians_per_count_;
    return;
  }
  commanding_ = true;
  commanded_position_ += velocity * period;
}

double WheelEncoder::following_error() const
{
  return commanded_position_ - static_cast<double>(count_) * radians_per_count_;
}

void WheelEncoder::check_following_error()
{
  const double error = std::abs(following_error());
  max_following_error_ = std::max(max_following_error_, error);
  if (config_.following_error_limit > 0.0 && error > config_.following_error_limit) {
    fault_ = true;
  }
}

}  // namespace mecanum_hardware
