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

#include "mecanum_lidar/l2_conversions.hpp"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_field.hpp"

namespace mecanum_lidar
{

namespace
{

sensor_msgs::msg::PointField make_field(
  const std::string & name, std::size_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = static_cast<std::uint32_t>(offset);
  field.datatype = datatype;
  field.count = 1;
  return field;
}

std::vector<sensor_msgs::msg::PointField> cloud_point_fields()
{
  using sensor_msgs::msg::PointField;
  return {
    make_field("x", offsetof(CloudPoint, x), PointField::FLOAT32),
    make_field("y", offsetof(CloudPoint, y), PointField::FLOAT32),
    make_field("z", offsetof(CloudPoint, z), PointField::FLOAT32),
    make_field("intensity", offsetof(CloudPoint, intensity), PointField::FLOAT32),
    make_field("time", offsetof(CloudPoint, time), PointField::FLOAT32),
    make_field("ring", offsetof(CloudPoint, ring), PointField::UINT16),
  };
}

}  // namespace

builtin_interfaces::msg::Time to_stamp(double seconds)
{
  double whole = std::floor(seconds);
  auto nanosec = static_cast<std::int64_t>(std::llround((seconds - whole) * 1e9));
  if (nanosec >= 1000000000) {
    whole += 1.0;
    nanosec -= 1000000000;
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(whole);
  stamp.nanosec = static_cast<std::uint32_t>(nanosec);
  return stamp;
}

sensor_msgs::msg::PointCloud2 to_cloud_msg(
  const unilidar_sdk2::PointCloudUnitree & cloud, const std::string & frame_id)
{
  static const std::vector<sensor_msgs::msg::PointField> fields = cloud_point_fields();

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = to_stamp(cloud.stamp);
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(cloud.points.size());
  msg.fields = fields;
  msg.is_bigendian = false;
  msg.point_step = sizeof(CloudPoint);
  msg.row_step = msg.point_step * msg.width;
  // The SDK drops returns with no echo or outside [range_min, range_max], so every point is
  // finite.
  msg.is_dense = true;
  msg.data.resize(msg.row_step);

  std::uint8_t * out = msg.data.data();
  for (const auto & p : cloud.points) {
    const CloudPoint point{
      p.x, p.y, p.z, p.intensity, p.time, static_cast<std::uint16_t>(p.ring), 0};
    std::memcpy(out, &point, sizeof(point));
    out += sizeof(point);
  }
  return msg;
}

sensor_msgs::msg::Imu to_imu_msg(
  const unilidar_sdk2::LidarImuData & imu, const std::string & frame_id)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp.sec = static_cast<std::int32_t>(imu.info.stamp.sec);
  msg.header.stamp.nanosec = imu.info.stamp.nsec;
  msg.header.frame_id = frame_id;

  // The SDK quaternion is (w, x, y, z). The upstream README's sample output labels it
  // "(x, y, z, w) = [-0.3645, 0.0077, 0.0099, 0.9293]" while the accelerometer reads
  // (0.22, 0.35, 9.58) m/s^2, so the sensor is within about 2.5 deg of level. Read as (w, x, y, z)
  // that quaternion is about 1.4 deg of tilt plus a yaw; read as (x, y, z, w) it is about 43 deg
  // of roll. The vendor ROS 1 node and the vendor ROS 2 TF broadcast use (w, x, y, z); the vendor
  // ROS 2 Imu message uses (x, y, z, w) and is wrong.
  msg.orientation.w = imu.quaternion[0];
  msg.orientation.x = imu.quaternion[1];
  msg.orientation.y = imu.quaternion[2];
  msg.orientation.z = imu.quaternion[3];

  msg.angular_velocity.x = imu.angular_velocity[0];
  msg.angular_velocity.y = imu.angular_velocity[1];
  msg.angular_velocity.z = imu.angular_velocity[2];

  msg.linear_acceleration.x = imu.linear_acceleration[0];
  msg.linear_acceleration.y = imu.linear_acceleration[1];
  msg.linear_acceleration.z = imu.linear_acceleration[2];

  // Covariances stay all zero, which REP 145 defines as "unknown". The datasheet gives none.
  return msg;
}

}  // namespace mecanum_lidar
