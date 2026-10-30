# mecanum_lidar

Driver for the Unitree L2 4D LiDAR, a bench-test launch file and an RViz configuration.

The driver `l2_driver` is a small C++ node built on Unitree's SDK
([third_party/unilidar_sdk2](third_party/unilidar_sdk2/README.md), vendored, BSD-3). It reads
the L2 over Ethernet (UDP, the default here) or USB serial, and publishes:

| Topic | Type | Frame | Content |
|---|---|---|---|
| `/l2/points` | `sensor_msgs/PointCloud2` | `l2_lidar` | One cloud per `cloud_scan_num` scans. Fields: `x y z intensity` (float32), `time` (float32, seconds after `header.stamp`), `ring` (uint16). 24 bytes per point. |
| `/l2/imu` | `sensor_msgs/Imu` | `l2_imu` | Built-in 6-axis IMU: orientation from the L2's own filter, angular velocity in rad/s, acceleration in m/s². Covariances are all zero, meaning unknown. |

Both publishers are reliable, so RViz and best-effort subscribers both connect.

The driver publishes **no TF**. `l2_lidar` and `l2_imu` come from robot_state_publisher, from
`mecanum_description/urdf/sensors/unitree_l2.xacro`.

## Why not Unitree's ROS 2 node

The vendor package `unitree_lidar_ros2` (v2.0.10) has these problems, all checked against its
source:

- **Wrong IMU orientation.** The SDK quaternion is ordered (w, x, y, z), but the node publishes
  it as (x, y, z, w). The evidence is in `src/l2_conversions.cpp` and is pinned by a unit test.
- **Conflicting TF.** It always broadcasts `<imu_frame>_initial -> <imu_frame>` and
  `<imu_frame> -> <cloud_frame>`. Both `l2_lidar` and `l2_imu` already have a parent in the URDF.
- **Writes the work mode on every start, defaulting to 0.** Over serial, that switches the L2
  back to Ethernet mode at its next power cycle. This driver writes it only when asked.
- **Swapped IP defaults.** The defaults in the node's code have the lidar and host addresses
  the wrong way round. Only its launch file has them right.
- **Polling.** It calls `runParse()` once per 1 ms wall-timer tick. This driver polls on its own
  thread.

The SDK has a bug of its own. `closeUDP()` recurses forever in `~UDPHandler` and segfaults, as
seen under gdb. The driver therefore never calls the close functions, and lets process exit
close the socket.

## USB serial setup (optional)

Connect the L2's UART plug to the computer with a USB-to-UART adapter that supports
**4,000,000 baud**. Many cheap adapters top out lower. The adapter normally appears as
`/dev/ttyACM0` or `/dev/ttyUSB0`; check with `ls /dev/ttyACM* /dev/ttyUSB*` after plugging it in.
Your user must be in the `dialout` group (`id -nG`). Log out and back in after adding it.

The L2 only streams over serial when work-mode bit 3 is set, and **the L2 ships in Ethernet
mode**. The documented way to switch it is over Ethernet: write work mode 8 there, then
power-cycle. If Ethernet does not work, try writing the mode over serial instead. It is
harmless if the L2 ignores it, but it is not documented to work:

```bash
ros2 run mecanum_lidar l2_driver --ros-args -p connection:=serial -p work_mode:=8 \
  -p serial_port:=/dev/ttyACM0
# Ctrl-C, power-cycle the L2, then run the bench test below.
```

If the status shows "Receiving data from the L2" before you even write the mode, the L2 is
already in serial mode. To go back to Ethernet later, run the same command with
`work_mode:=0`, then power-cycle.

## Ethernet setup (default)

Ethernet is the default (`connection:=udp`). On the AMD Renoir/Cezanne dev laptop, sustained
4 Mbaud USB-serial traffic made the USB host controller crash ("xHCI host controller not
responding, assume dead"), taking every USB-A port down until reboot. Ethernet avoids USB
entirely and is the L2's native interface.


Factory settings: the L2 is `192.168.1.62` and sends UDP from port 6101 to
`192.168.1.2:6201`. The computer must have **192.168.1.2** on the interface the L2 is plugged
into. The SDK listens on port 6201 on every interface, so a wrong address does not fail at
startup. It shows up as the driver's "No data from the L2" warning.

With NetworkManager, find your Ethernet interface with `ip -br link`. It is the `enp…` or
`eth…` entry, and differs between machines. Set `IFACE` to it.

The profile uses **host routes (/32)** instead of a /24 subnet. It gives the computer only
192.168.1.2 on that port, with one route to the L2 at 192.168.1.62. Many home and office
networks, including Wi-Fi, also use 192.168.1.0/24. A /24 on the Ethernet port would clash with
them. The /32 routes are more specific, so traffic to the L2 goes over the cable and everything
else stays on Wi-Fi. The L2 keeps its factory addresses.

```bash
IFACE=enp2s0   # replace with yours
nmcli connection add type ethernet ifname "$IFACE" con-name unitree-l2-eth \
  ipv4.method manual ipv4.addresses 192.168.1.2/32 ipv4.routes "192.168.1.62/32" \
  ipv4.never-default yes ipv6.method disabled connection.autoconnect yes \
  connection.autoconnect-priority 10
```

The profile activates on its own when the port gets a link. If `ip -br link` shows the port as
`NO-CARRIER`, the L2 is unplugged, unpowered or still booting. Check the link and the route with:

```bash
ip -br addr show "$IFACE"            # 192.168.1.2/32
ip route get 192.168.1.62            # ... dev enp2s0 src 192.168.1.2
ping -c 3 192.168.1.62
```

If 192.168.1.2 or .62 is already used by a device on your Wi-Fi network, that device becomes
unreachable from this computer while the profile is up. In that case, move the L2 to its own
subnet with the `set_ip_address` example from upstream unilidar_sdk2 (not vendored here), then
pass `lidar_ip:=` and `local_ip:=`.

To see whether packets arrive at all, run `sudo tcpdump -ni "$IFACE" udp port 6201`.

## Bench test

The L2 sits on a desk, powered and connected over Ethernet (or USB serial with `connection:=serial`).

```bash
source install/setup.bash
ros2 launch mecanum_lidar l2.launch.py                                               # Ethernet
ros2 launch mecanum_lidar l2.launch.py connection:=serial serial_port:=/dev/ttyACM0  # USB-UART
```

RViz opens with the fixed frame `l2_lidar`. The cloud is coloured by height, with 1 s decay so
the room fills in. What to check:

1. **The driver log** says `Receiving data from the L2.` Every 5 s it then prints a status line:
   `cloud … Hz, … points/cloud | imu … Hz, mean accel (x, y, z) |a| m/s^2 | cover dirt … %`.
   The vendor's sample output implies about 12 clouds/s of about 5200 points with the default
   `cloud_scan_num: 18`.
2. **IMU axes and units.** With the L2 upright on a level desk, the mean acceleration should be
   about (0, 0, +9.8) m/s². Raising the +X side (away from the cable) should make the X
   reading positive.
3. **Frame direction.** The red axis (+X) should point away from the cable outlet. Objects in
   front of that side should appear along the red axis. The URDF's `l2_mount -> l2_lidar`
   rotation assumes this convention. On the robot that rotation is `l2_cable_yaw` in
   `unitree_l2.xacro`, a stand-in of 0 (cable toward the rear). Read the cable direction on the
   mounted L2 and set it.
4. **Cover dirt** should stay near 0 %. A high value means the dome needs cleaning.

You can also check the rates with `ros2 topic hz /l2/points` and `ros2 topic hz /l2/imu`.

For the full hardware procedure, see
[docs/hardware_l2_test.md](../../docs/hardware_l2_test.md). It covers setting the work mode,
`cloud_scan_num` and ranges to match the simulation, verifying the mount with
`tools/l2_mount_check.py`, and the sim-against-real comparison.

### Mounted on the robot

```bash
ros2 launch mecanum_lidar l2.launch.py robot_model:=true fixed_frame:=base_footprint
```

This also starts robot_state_publisher and joint_state_publisher with the full robot description.
In RViz, tick **RobotModel**. The L2 stands upright on the robot's top at 0.80 m, so it sees
the walls above the robot and the ceiling, and the floor only far away. The ceiling should be
level and the walls vertical and straight. If they are not, the mounted L2 does not match the
mount in `mecanum_description/urdf/sensors/unitree_l2.xacro`. Check the mount height, its tilt
and the cable direction; `tools/l2_mount_check.py` measures them.

Leave `robot_model:=false` when another launch file already publishes the robot description.

## Launch arguments (`l2.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `connection` | `udp` | `udp` (Ethernet) or `serial` (USB) |
| `lidar_ip` | `192.168.1.62` | L2 address (udp) |
| `local_ip` | `192.168.1.2` | This computer's address on the L2 link (udp) |
| `serial_port` | `/dev/ttyACM0` | Serial device (serial) |
| `params_file` | `config/l2.yaml` | All other driver parameters |
| `rviz` | `true` | Start RViz with `rviz/l2.rviz` |
| `robot_model` | `false` | Also start robot_state_publisher and joint_state_publisher |
| `fixed_frame` | `l2_lidar` | RViz fixed frame. `base_footprint` needs `robot_model:=true`. |

## Driver parameters (`config/l2.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `lidar_port`, `local_port` | `6101`, `6201` | UDP ports, factory defaults |
| `baudrate` | `4000000` | Serial speed, factory default |
| `work_mode` | `-1` | `-1` leaves the stored mode unchanged. Otherwise it is a bit field (FOV, 2D/3D, IMU, Ethernet/serial, auto-start); the bits are documented in the YAML. |
| `cloud_scan_num` | `18` | Scans per published cloud |
| `range_min`, `range_max` | `0.0`, `100.0` m | The SDK drops returns outside this range. The defaults do no filtering. |
| `use_system_timestamp` | `true` | Stamp with this computer's clock. `false` uses the L2's unsynchronised clock. |
| `cloud_frame_id`, `imu_frame_id` | `l2_lidar`, `l2_imu` | Frames from the URDF |
| `data_timeout` | `2.0` s | Warn after this long without packets |
| `stats_period` | `5.0` s | Interval between status lines. `0` disables them. |

## Verification without hardware

The driver was tested over loopback against a synthetic L2. That was a scratch program that
built protocol packets with the SDK's own structs and sent a box-shaped room plus a level IMU.
The results:

- 12 clouds/s of 5400 points, and 250 IMU messages/s, matching what was sent.
- Every point within 0.5 mm of the room's walls (the 1 mm range resolution), in `l2_lidar`.
- The IMU orientation came out exactly as sent, in (w, x, y, z) order.
- TF `base_footprint -> l2_lidar` and `l2_lidar -> l2_imu` match the URDF.
- Clean exit on Ctrl-C.

That synthetic test could not check the vendor firmware's real packet timing, the `ring` values
or the RViz rendering. In loopback, the SDK set `ring` to 1 for every point.
