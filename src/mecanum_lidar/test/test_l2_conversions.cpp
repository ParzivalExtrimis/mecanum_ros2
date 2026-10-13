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
#include <map>
#include <string>

#include "mecanum_lidar/l2_conversions.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

unilidar_sdk2::PointUnitree make_point(float x, float y, float z, float time, std::uint32_t ring)
{
  unilidar_sdk2::PointUnitree p{};
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 7.0F;
  p.time = time;
  p.ring = ring;
  return p;
}

// Sample IMU packet from the unilidar_sdk2 README (sensor resting near level).
unilidar_sdk2::LidarImuData readme_imu_sample()
{
  unilidar_sdk2::LidarImuData imu{};
  imu.info.stamp.sec = 1730191291;
  imu.info.stamp.nsec = 304411172;
  imu.quaternion[0] = -0.3645F;
  imu.quaternion[1] = 0.0077F;
  imu.quaternion[2] = 0.0099F;
  imu.quaternion[3] = 0.9293F;
  imu.angular_velocity[0] = 0.0209F;
  imu.angular_velocity[1] = -0.0644F;
  imu.angular_velocity[2] = 0.0146F;
  imu.linear_acceleration[0] = 0.2215F;
  imu.linear_acceleration[1] = 0.3537F;
  imu.linear_acceleration[2] = 9.5822F;
  return imu;
}

}  // namespace

TEST(ToStamp, SplitsSecondsAndNanoseconds)
{
  const auto stamp = mecanum_lidar::to_stamp(1730724860.502892);
  EXPECT_EQ(stamp.sec, 1730724860);
  // A double holds a 2024 epoch time to about 0.24 us.
  EXPECT_NEAR(static_cast<double>(stamp.nanosec), 502892000.0, 1000.0);
}

TEST(ToStamp, CarriesRoundedNanosecondsIntoSeconds)
{
  const auto stamp = mecanum_lidar::to_stamp(41.9999999999);
  EXPECT_EQ(stamp.sec, 42);
  EXPECT_EQ(stamp.nanosec, 0U);
}

TEST(ToImuMsg, ReadsQuaternionAsWxyz)
{
  const auto msg = mecanum_lidar::to_imu_msg(readme_imu_sample(), "l2_imu");
  EXPECT_FLOAT_EQ(msg.orientation.w, -0.3645F);
  EXPECT_FLOAT_EQ(msg.orientation.x, 0.0077F);
  EXPECT_FLOAT_EQ(msg.orientation.y, 0.0099F);
  EXPECT_FLOAT_EQ(msg.orientation.z, 0.9293F);

  // The orientation must agree with the accelerometer: both say the sensor is close to level.
  const auto & q = msg.orientation;
  const double norm2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  const double body_z_up = 1.0 - 2.0 * (q.x * q.x + q.y * q.y) / norm2;
  const double tilt_deg = std::acos(body_z_up) * 180.0 / M_PI;
  const auto & a = msg.linear_acceleration;
  const double accel_tilt_deg = std::atan2(std::hypot(a.x, a.y), a.z) * 180.0 / M_PI;
  EXPECT_LT(tilt_deg, 5.0);
  EXPECT_NEAR(tilt_deg, accel_tilt_deg, 2.0);
}

TEST(ToImuMsg, CopiesRatesAccelerationsStampAndFrame)
{
  const auto msg = mecanum_lidar::to_imu_msg(readme_imu_sample(), "l2_imu");
  EXPECT_EQ(msg.header.frame_id, "l2_imu");
  EXPECT_EQ(msg.header.stamp.sec, 1730191291);
  EXPECT_EQ(msg.header.stamp.nanosec, 304411172U);
  EXPECT_FLOAT_EQ(msg.angular_velocity.x, 0.0209F);
  EXPECT_FLOAT_EQ(msg.angular_velocity.y, -0.0644F);
  EXPECT_FLOAT_EQ(msg.angular_velocity.z, 0.0146F);
  EXPECT_FLOAT_EQ(msg.linear_acceleration.x, 0.2215F);
  EXPECT_FLOAT_EQ(msg.linear_acceleration.y, 0.3537F);
  EXPECT_FLOAT_EQ(msg.linear_acceleration.z, 9.5822F);
}

TEST(ToCloudMsg, DeclaresVendorFieldsInA24BytePoint)
{
  unilidar_sdk2::PointCloudUnitree cloud{};
  const auto msg = mecanum_lidar::to_cloud_msg(cloud, "l2_lidar");

  using sensor_msgs::msg::PointField;
  const std::map<std::string, std::uint8_t> expected{
    {"x", PointField::FLOAT32}, {"y", PointField::FLOAT32}, {"z", PointField::FLOAT32},
    {"intensity", PointField::FLOAT32}, {"time", PointField::FLOAT32},
    {"ring", PointField::UINT16}};
  ASSERT_EQ(msg.fields.size(), expected.size());
  for (const auto & field : msg.fields) {
    ASSERT_EQ(expected.count(field.name), 1U) << field.name;
    EXPECT_EQ(field.datatype, expected.at(field.name)) << field.name;
    EXPECT_EQ(field.count, 1U) << field.name;
  }
  EXPECT_EQ(msg.point_step, 24U);
  EXPECT_EQ(msg.width, 0U);
  EXPECT_EQ(msg.height, 1U);
  EXPECT_TRUE(msg.data.empty());
}

TEST(ToCloudMsg, CopiesPointsStampAndFrame)
{
  unilidar_sdk2::PointCloudUnitree cloud{};
  cloud.stamp = 1730724860.25;
  cloud.points.push_back(make_point(1.5F, -1.0F, 0.0F, 0.0F, 1));
  cloud.points.push_back(make_point(-0.5F, 0.25F, 2.0F, 0.0042F, 18));

  const auto msg = mecanum_lidar::to_cloud_msg(cloud, "l2_lidar");
  EXPECT_EQ(msg.header.frame_id, "l2_lidar");
  EXPECT_EQ(msg.header.stamp.sec, 1730724860);
  EXPECT_EQ(msg.header.stamp.nanosec, 250000000U);
  ASSERT_EQ(msg.width, 2U);
  EXPECT_EQ(msg.row_step, 48U);
  ASSERT_EQ(msg.data.size(), 48U);

  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> z(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> intensity(msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<float> time(msg, "time");
  sensor_msgs::PointCloud2ConstIterator<std::uint16_t> ring(msg, "ring");
  EXPECT_FLOAT_EQ(*x, 1.5F);
  EXPECT_FLOAT_EQ(*intensity, 7.0F);
  EXPECT_EQ(*ring, 1U);
  ++x;
  ++z;
  ++time;
  ++ring;
  EXPECT_FLOAT_EQ(*x, -0.5F);
  EXPECT_FLOAT_EQ(*z, 2.0F);
  EXPECT_FLOAT_EQ(*time, 0.0042F);
  EXPECT_EQ(*ring, 18U);
}
