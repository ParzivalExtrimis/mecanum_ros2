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

// Unitree L2 driver node.
//
// Reads the L2 over USB serial (default) or Ethernet (UDP) through the vendored SDK
// (third_party/unilidar_sdk2) and publishes:
//   l2/points  sensor_msgs/PointCloud2  frame cloud_frame_id (l2_lidar)
//              fields x y z intensity time ring; time is float32 seconds after header.stamp
//   l2/imu     sensor_msgs/Imu          frame imu_frame_id (l2_imu)
//
// It publishes no TF. The l2_lidar and l2_imu frames come from robot_state_publisher
// (mecanum_description/urdf/sensors/unitree_l2.xacro), so each frame has exactly one parent.
//
// The SDK is not thread-safe, so every SDK call after construction happens on the reader thread.
// The status timer only reads counters.

#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "mecanum_lidar/l2_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "unilidar_sdk2/unitree_lidar_sdk.h"

namespace mecanum_lidar
{

namespace
{

std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::uint16_t checked_port(std::int64_t port, const std::string & name)
{
  if (port < 1 || port > 65535) {
    throw std::invalid_argument(name + " must be in 1..65535, got " + std::to_string(port));
  }
  return static_cast<std::uint16_t>(port);
}

}  // namespace

class L2DriverNode : public rclcpp::Node
{
public:
  explicit L2DriverNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("l2_driver", options)
  {
    connection_ = declare_parameter<std::string>("connection", "udp");
    lidar_ip_ = declare_parameter<std::string>("lidar_ip", "192.168.1.62");
    lidar_port_ = checked_port(declare_parameter<std::int64_t>("lidar_port", 6101), "lidar_port");
    local_ip_ = declare_parameter<std::string>("local_ip", "192.168.1.2");
    local_port_ = checked_port(declare_parameter<std::int64_t>("local_port", 6201), "local_port");
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    baudrate_ = declare_parameter<std::int64_t>("baudrate", 4000000);
    work_mode_ = declare_parameter<std::int64_t>("work_mode", -1);
    cloud_scan_num_ = declare_parameter<std::int64_t>("cloud_scan_num", 18);
    range_min_ = declare_parameter<double>("range_min", 0.0);
    range_max_ = declare_parameter<double>("range_max", 100.0);
    use_system_timestamp_ = declare_parameter<bool>("use_system_timestamp", true);
    cloud_frame_id_ = declare_parameter<std::string>("cloud_frame_id", "l2_lidar");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "l2_imu");
    data_timeout_ = declare_parameter<double>("data_timeout", 2.0);
    stats_period_ = declare_parameter<double>("stats_period", 5.0);

    if (cloud_scan_num_ < 1 || cloud_scan_num_ > 65535) {
      throw std::invalid_argument("cloud_scan_num must be in 1..65535");
    }
    if (baudrate_ < 1 || baudrate_ > UINT32_MAX) {
      throw std::invalid_argument("baudrate must be positive");
    }
    if (!(range_min_ >= 0.0 && range_max_ > range_min_)) {
      throw std::invalid_argument("need 0 <= range_min < range_max");
    }
    if (work_mode_ < -1 || work_mode_ > UINT32_MAX) {
      throw std::invalid_argument("work_mode must be -1 (leave unchanged) or a 32-bit bit field");
    }
    if (!use_system_timestamp_) {
      RCLCPP_WARN(
        get_logger(), "use_system_timestamp is false: stamps come from the L2's own clock, which "
        "is not synchronised with this computer, so TF lookups and LIO will fail.");
    }

    // Reliable, so the default RViz subscription (reliable) and best-effort subscribers both
    // connect. The IMU depth covers about 0.2 s of samples for a briefly stalled consumer.
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("l2/points", rclcpp::QoS(5));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("l2/imu", rclcpp::QoS(50));

    connect();

    start_ns_ = steady_now_ns();
    stats_start_ns_ = start_ns_;
    running_ = true;
    reader_thread_ = std::thread([this]() {read_loop();});
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {on_status_timer();});
  }

  ~L2DriverNode() override
  {
    running_ = false;
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    // The reader is deliberately not closed or deleted. In unilidar_sdk2 v2.0.10,
    // UnitreeLidarReaderImp::closeUDP() recurses forever in ~UDPHandler and segfaults (seen
    // under gdb), the vendor node never calls the close functions either, and the SDK has no
    // destroy function. Process exit closes the socket or serial port.
  }

private:
  struct Stats
  {
    std::uint64_t clouds{0};
    std::uint64_t points{0};
    std::uint64_t imus{0};
    std::array<double, 3> accel_sum{0.0, 0.0, 0.0};
    float dirty_percent{NAN};
  };

  void connect()
  {
    reader_ = unilidar_sdk2::createUnitreeLidarReader();
    std::string sdk_version;
    if (reader_->getVersionOfSDK(sdk_version)) {
      RCLCPP_INFO(get_logger(), "unilidar_sdk2 version %s", sdk_version.c_str());
    }

    const auto scan_num = static_cast<std::uint16_t>(cloud_scan_num_);
    const auto range_min = static_cast<float>(range_min_);
    const auto range_max = static_cast<float>(range_max_);
    int rc = 0;
    if (connection_ == "udp") {
      rc = reader_->initializeUDP(
        lidar_port_, lidar_ip_, local_port_, local_ip_, scan_num, use_system_timestamp_,
        range_min, range_max);
    } else if (connection_ == "serial") {
      rc = reader_->initializeSerial(
        serial_port_, static_cast<std::uint32_t>(baudrate_), scan_num, use_system_timestamp_,
        range_min, range_max);
    } else {
      throw std::invalid_argument(
              "connection must be 'udp' or 'serial', got '" + connection_ + "'");
    }
    if (rc != 0) {
      throw std::runtime_error("Could not open the L2 connection. " + connection_hint());
    }
    RCLCPP_INFO(get_logger(), "Opened %s. Waiting for data.", connection_description().c_str());

    if (work_mode_ >= 0) {
      const bool serial_bit = (work_mode_ & 0x8) != 0;
      if (serial_bit != (connection_ == "serial")) {
        RCLCPP_WARN(
          get_logger(), "work_mode %" PRId64 " selects %s mode but this session uses %s. After its "
          "next power cycle the L2 will only talk over %s.", work_mode_,
          serial_bit ? "serial" : "Ethernet", connection_.c_str(),
          serial_bit ? "serial" : "Ethernet");
      }
      reader_->setLidarWorkMode(static_cast<std::uint32_t>(work_mode_));
      RCLCPP_INFO(
        get_logger(), "Wrote work mode %" PRId64 " to the L2. It is stored in the L2; the "
        "Ethernet/serial bit takes effect after a power cycle.", work_mode_);
    }

    // Needed when work-mode bit 4 makes the L2 wait for a start command after power-on; a no-op
    // when it is already spinning.
    reader_->startLidarRotation();
  }

  void read_loop()
  {
    unilidar_sdk2::PointCloudUnitree cloud;
    unilidar_sdk2::LidarImuData imu;
    try {
      while (running_ && rclcpp::ok()) {
        const int packet_type = reader_->runParse();
        if (packet_type == 0) {
          std::this_thread::sleep_for(std::chrono::microseconds(200));
          continue;
        }
        last_packet_ns_ = steady_now_ns();

        if (packet_type == LIDAR_POINT_DATA_PACKET_TYPE) {
          if (reader_->getPointCloud(cloud)) {
            cloud_pub_->publish(to_cloud_msg(cloud, cloud_frame_id_));
            float dirty = NAN;
            const bool have_dirty = reader_->getDirtyPercentage(dirty);
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.clouds;
            stats_.points += cloud.points.size();
            if (have_dirty) {
              stats_.dirty_percent = dirty;
            }
          }
        } else if (packet_type == LIDAR_IMU_DATA_PACKET_TYPE) {
          if (reader_->getImuData(imu)) {
            imu_pub_->publish(to_imu_msg(imu, imu_frame_id_));
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.imus;
            for (std::size_t i = 0; i < 3; ++i) {
              stats_.accel_sum[i] += imu.linear_acceleration[i];
            }
          }
        } else if (packet_type == LIDAR_VERSION_PACKET_TYPE && !versions_logged_) {
          std::string firmware;
          std::string hardware;
          if (reader_->getVersionOfLidarFirmware(firmware) &&
            reader_->getVersionOfLidarHardware(hardware))
          {
            RCLCPP_INFO(
              get_logger(), "L2 firmware %s, hardware %s", firmware.c_str(), hardware.c_str());
            versions_logged_ = true;
          }
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "L2 reader thread stopped: %s", e.what());
    }
  }

  void on_status_timer()
  {
    const std::int64_t now = steady_now_ns();
    const std::int64_t last = last_packet_ns_;
    const double silent_s = 1e-9 * static_cast<double>(now - (last == 0 ? start_ns_ : last));

    if (silent_s > data_timeout_) {
      if (receiving_) {
        RCLCPP_WARN(get_logger(), "Data from the L2 stopped.");
        receiving_ = false;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 10000, "No data from the L2 for %.0f s. %s", silent_s,
        connection_hint().c_str());
      return;
    }
    if (last == 0) {
      return;  // Still within data_timeout of startup.
    }
    if (!receiving_) {
      RCLCPP_INFO(get_logger(), "Receiving data from the L2.");
      receiving_ = true;
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_ = Stats();
      stats_start_ns_ = now;
      return;
    }

    const double elapsed_s = 1e-9 * static_cast<double>(now - stats_start_ns_);
    // Half a timer tick of slack, so a 5 s period does not slip to 6 s on timer jitter.
    if (stats_period_ <= 0.0 || elapsed_s + 0.5 < stats_period_) {
      return;
    }
    Stats stats;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats = stats_;
      stats_ = Stats();
    }
    stats_start_ns_ = now;

    const double points_per_cloud =
      stats.clouds > 0 ? static_cast<double>(stats.points) / static_cast<double>(stats.clouds) :
      0.0;
    std::array<double, 3> accel{0.0, 0.0, 0.0};
    if (stats.imus > 0) {
      for (std::size_t i = 0; i < 3; ++i) {
        accel[i] = stats.accel_sum[i] / static_cast<double>(stats.imus);
      }
    }
    const double accel_norm = std::sqrt(
      accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    char dirt[32] = "unknown";
    if (!std::isnan(stats.dirty_percent)) {
      std::snprintf(dirt, sizeof(dirt), "%.1f %%", static_cast<double>(stats.dirty_percent));
    }
    RCLCPP_INFO(
      get_logger(),
      "cloud %.1f Hz, %.0f points/cloud | imu %.1f Hz, mean accel (%.2f, %.2f, %.2f) |%.2f| m/s^2 "
      "| cover dirt %s",
      static_cast<double>(stats.clouds) / elapsed_s, points_per_cloud,
      static_cast<double>(stats.imus) / elapsed_s, accel[0], accel[1], accel[2], accel_norm, dirt);
  }

  std::string connection_description() const
  {
    if (connection_ == "udp") {
      return "UDP " + local_ip_ + ":" + std::to_string(local_port_) + " <- L2 " + lidar_ip_ +
             ":" + std::to_string(lidar_port_);
    }
    return "serial " + serial_port_ + " at " + std::to_string(baudrate_) + " baud";
  }

  std::string connection_hint() const
  {
    if (connection_ == "udp") {
      return "Expecting " + connection_description() + ". Check that the L2 is powered, the "
             "Ethernet cable is in, and " + local_ip_ + " is assigned to the host interface the "
             "L2 is plugged into (see the mecanum_lidar README, 'Network setup').";
    }
    return "Expecting " + connection_description() + ". Check the cable, that the L2 is in "
           "serial mode (work mode bit 3), and that your user is in the dialout group.";
  }

  // Parameters
  std::string connection_;
  std::string lidar_ip_;
  std::uint16_t lidar_port_{0};
  std::string local_ip_;
  std::uint16_t local_port_{0};
  std::string serial_port_;
  std::int64_t baudrate_{0};
  std::int64_t work_mode_{-1};
  std::int64_t cloud_scan_num_{0};
  double range_min_{0.0};
  double range_max_{0.0};
  bool use_system_timestamp_{true};
  std::string cloud_frame_id_;
  std::string imu_frame_id_;
  double data_timeout_{0.0};
  double stats_period_{0.0};

  unilidar_sdk2::UnitreeLidarReader * reader_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::thread reader_thread_;
  std::atomic<bool> running_{false};
  std::atomic<std::int64_t> last_packet_ns_{0};  // steady clock; 0 = no packet yet
  bool versions_logged_{false};  // reader thread only

  // Status timer only
  std::int64_t start_ns_{0};
  std::int64_t stats_start_ns_{0};
  bool receiving_{false};

  std::mutex stats_mutex_;
  Stats stats_;
};

}  // namespace mecanum_lidar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<mecanum_lidar::L2DriverNode>(rclcpp::NodeOptions()));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("l2_driver"), "%s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
