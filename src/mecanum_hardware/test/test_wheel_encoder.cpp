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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "mecanum_hardware/wheel_encoder.hpp"

namespace
{

using mecanum_hardware::WheelEncoder;
using mecanum_hardware::WheelEncoderConfig;

constexpr double kTwoPi = 6.283185307179586;
constexpr double kPeriod = 0.02;

WheelEncoderConfig config(int delay = 0, double limit = 0.0)
{
  WheelEncoderConfig c;
  c.counts_per_rev = 4000;
  c.report_delay_cycles = delay;
  c.following_error_limit = limit;
  return c;
}

TEST(WheelEncoder, QuantizesWithFloor)
{
  const double count_angle = kTwoPi / 4000.0;
  EXPECT_EQ(WheelEncoder::quantize(0.0, 4000), 0);
  EXPECT_EQ(WheelEncoder::quantize(0.99 * count_angle, 4000), 0);
  EXPECT_EQ(WheelEncoder::quantize(1.01 * count_angle, 4000), 1);
  EXPECT_EQ(WheelEncoder::quantize(-0.01 * count_angle, 4000), -1);
  EXPECT_EQ(WheelEncoder::quantize(10.0 * kTwoPi + 0.5 * count_angle, 4000), 40000);
}

TEST(WheelEncoder, PositionIsTheCountInRadians)
{
  WheelEncoder encoder(config());
  encoder.update(1234, kPeriod);
  EXPECT_DOUBLE_EQ(encoder.position(), 1234 * kTwoPi / 4000.0);
  EXPECT_DOUBLE_EQ(encoder.velocity(), 0.0);  // no previous count yet
}

TEST(WheelEncoder, IntegratedVelocityReproducesTheCount)
{
  // A wheel at an uneven speed with uneven periods: the velocity times its period, summed as
  // the drive controller does, must land exactly on the last count.
  WheelEncoder encoder(config());
  double exact = 0.3;
  encoder.update(WheelEncoder::quantize(exact, 4000), kPeriod);
  const double start = encoder.position();
  double integrated = 0.0;
  for (int i = 0; i < 500; ++i) {
    const double period = kPeriod * (1.0 + 0.3 * std::sin(0.7 * i));
    exact += (2.9 + 1.5 * std::sin(0.05 * i)) * period;
    encoder.update(WheelEncoder::quantize(exact, 4000), period);
    integrated += encoder.velocity() * period;
  }
  EXPECT_NEAR(start + integrated, encoder.position(), 1e-9);
  EXPECT_NEAR(encoder.position(), exact, kTwoPi / 4000.0);
}

TEST(WheelEncoder, NegativeMotion)
{
  WheelEncoder encoder(config());
  encoder.update(0, kPeriod);
  encoder.update(-37, kPeriod);
  EXPECT_DOUBLE_EQ(encoder.velocity(), -37 * kTwoPi / 4000.0 / kPeriod);
}

TEST(WheelEncoder, ReportDelay)
{
  WheelEncoder encoder(config(2));
  encoder.update(0, kPeriod);
  encoder.update(10, kPeriod);
  EXPECT_DOUBLE_EQ(encoder.position(), 0.0);   // held until two newer counts exist
  encoder.update(30, kPeriod);
  EXPECT_DOUBLE_EQ(encoder.position(), 0.0);
  EXPECT_DOUBLE_EQ(encoder.velocity(), 0.0);
  encoder.update(60, kPeriod);
  EXPECT_DOUBLE_EQ(encoder.position(), 10 * kTwoPi / 4000.0);
  EXPECT_DOUBLE_EQ(encoder.velocity(), 10 * kTwoPi / 4000.0 / kPeriod);
  encoder.update(100, kPeriod);
  EXPECT_DOUBLE_EQ(encoder.velocity(), 20 * kTwoPi / 4000.0 / kPeriod);
  EXPECT_EQ(encoder.count(), 100);  // the undelayed count
}

TEST(WheelEncoder, FollowingErrorStaysSmallWhenTracking)
{
  WheelEncoder encoder(config(1, 0.1));
  double exact = 0.0;
  encoder.update(0, kPeriod);
  for (int i = 0; i < 1000; ++i) {
    encoder.command(2.0, kPeriod);
    exact += 2.0 * kPeriod;
    encoder.update(WheelEncoder::quantize(exact, 4000), kPeriod);
  }
  EXPECT_LT(encoder.max_following_error(), 2.0 * kTwoPi / 4000.0);
  EXPECT_FALSE(encoder.fault());
}

TEST(WheelEncoder, FrozenCountFaults)
{
  WheelEncoder encoder(config(0, 0.1));
  encoder.update(0, kPeriod);
  int cycles = 0;
  while (!encoder.fault() && cycles < 100) {
    encoder.command(2.0, kPeriod);   // 0.04 rad per cycle commanded, nothing counted
    encoder.update(0, kPeriod);
    ++cycles;
  }
  EXPECT_TRUE(encoder.fault());
  EXPECT_EQ(cycles, 3);
  encoder.update(0, kPeriod);
  EXPECT_TRUE(encoder.fault());  // latched
}

TEST(WheelEncoder, NoCommandRezeroes)
{
  WheelEncoder encoder(config(0, 0.1));
  encoder.update(0, kPeriod);
  encoder.update(500, kPeriod);       // pushed by hand, no controller
  encoder.command(std::nan(""), kPeriod);
  encoder.update(500, kPeriod);
  EXPECT_FALSE(encoder.fault());
  EXPECT_DOUBLE_EQ(encoder.following_error(), 0.0);
}

TEST(WheelEncoder, LimitOffNeverFaults)
{
  WheelEncoder encoder(config(0, 0.0));
  encoder.update(0, kPeriod);
  encoder.command(100.0, 1.0);
  encoder.update(0, kPeriod);
  EXPECT_FALSE(encoder.fault());
}

TEST(WheelEncoder, RejectsBadConfig)
{
  WheelEncoderConfig bad = config();
  bad.counts_per_rev = 0;
  EXPECT_THROW(WheelEncoder{bad}, std::invalid_argument);
  bad = config(-1);
  EXPECT_THROW(WheelEncoder{bad}, std::invalid_argument);
}

}  // namespace
