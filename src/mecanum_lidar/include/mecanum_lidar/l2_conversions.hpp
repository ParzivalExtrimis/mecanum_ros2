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

#ifndef MECANUM_LIDAR__L2_CONVERSIONS_HPP_
#define MECANUM_LIDAR__L2_CONVERSIONS_HPP_

#include <cstdint>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "unilidar_sdk2/unitree_lidar_sdk.h"

namespace mecanum_lidar
{

/// One point of the published cloud. The field names and types match the vendor driver
/// (x y z intensity ring time), so LIO front ends written for the L2 read it unchanged. It is
/// packed into 24 bytes instead of the 48 bytes of the vendor's PCL point type.
struct CloudPoint
{
  float x;
  float y;
  float z;
  float intensity;
  float time;  ///< Seconds after the cloud's header.stamp.
  std::uint16_t ring;
  std::uint16_t padding;
};

static_assert(sizeof(CloudPoint) == 24, "CloudPoint must be 24 bytes");

/// Convert an SDK time in seconds (double) to a ROS stamp.
builtin_interfaces::msg::Time to_stamp(double seconds);

/// Convert an SDK cloud to a PointCloud2 with CloudPoint fields. header.stamp is the cloud's
/// start time; each point's `time` field is relative to it.
sensor_msgs::msg::PointCloud2 to_cloud_msg(
  const unilidar_sdk2::PointCloudUnitree & cloud, const std::string & frame_id);

/// Convert an SDK IMU sample to an Imu message. The SDK stores the orientation quaternion as
/// (w, x, y, z); see the implementation for the evidence.
sensor_msgs::msg::Imu to_imu_msg(
  const unilidar_sdk2::LidarImuData & imu, const std::string & frame_id);

}  // namespace mecanum_lidar

#endif  // MECANUM_LIDAR__L2_CONVERSIONS_HPP_
