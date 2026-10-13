# Vendored Unitree L2 SDK (unilidar_sdk2)

The C++ SDK for the Unitree L2 is vendored here so `mecanum_lidar` builds offline, for
example on the Jetson. Only the parts the driver needs are copied.

| Item | Value |
|---|---|
| Upstream | https://github.com/unitreerobotics/unilidar_sdk2 |
| Tag | `v2.0.10`, commit `0e3c51f512e6b8ff60b8c32f160b412cb48445c2` (2025-03-10) |
| License | BSD 3-Clause, see [LICENSE](LICENSE) |
| `include/unilidar_sdk2/` | `unitree_lidar_sdk.h`, `unitree_lidar_utilities.h`, `unitree_lidar_sdk_config.h`, `unitree_lidar_protocol.h`, copied unchanged from `unitree_lidar_sdk/include/`. The vendor examples, PCL helper and UDP handler headers are left out. |
| `lib/<arch>/libunilidar_sdk2.a` | Prebuilt static libraries, copied unchanged from `unitree_lidar_sdk/lib/`. Only `x86_64` and `aarch64` are provided upstream. |

`unitree_lidar_sdk_config.h` says `2.0.9` even though the tag is `v2.0.10`. The driver logs the
version that the library itself reports at startup.

SHA-256 of the libraries:

```
a8530545cbd435e3a96b9a099cd57921202aed3d690066f55ea0ade4310212d3  lib/x86_64/libunilidar_sdk2.a
4e334b67c1a92152c89363d8014a6e361d7bf590e58484d7d6ddc8541389de28  lib/aarch64/libunilidar_sdk2.a
```

## Updating

1. Clone the new upstream tag.
2. Copy the four headers and both `libunilidar_sdk2.a` files over the ones here.
3. Update the tag, commit and hashes in this file.
4. Rebuild and run `colcon test --packages-select mecanum_lidar`. Check the IMU quaternion order
   in `src/l2_conversions.cpp` against the new upstream README.

The vendor's own ROS 2 node (`unitree_lidar_ros2`) is not used. See the package README for why.
