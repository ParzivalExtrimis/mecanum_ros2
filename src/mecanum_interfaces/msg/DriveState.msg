# The drive ESP32's state as reported to the hardware plugin (mecanum_hardware/Esp32System),
# published on /drive/state for every report (100 Hz). Wheel order: front_left, front_right,
# rear_left, rear_right. Counts and pulses are in the wheel convention: positive rolls the robot
# forward. See docs/hardware_drive_test.md, "Protocol".

# Host receive time.
std_msgs/Header header

# The firmware's report counter and its clock (microseconds since boot).
uint16 seq
uint64 mcu_time_us

# Encoder counts through the tap, 64-bit, continuous.
int64[4] counts
# Step pulses emitted by the firmware.
int64[4] pulses
# Emitted pulses scaled to counts, minus counts, since the last configuration or fault clear.
int32[4] following_error

# The protocol's fault bits (0-3 following error per wheel, 4 watchdog, 5 no config,
# 6 overspeed, 7 latched, 8 step driver) and mode (0 no config, 1 idle, 2 running, 3 fault).
uint16 faults
uint8 mode
# Bad frames the firmware received, and bad frames the host received.
uint16 mcu_rx_errors
uint32 host_rx_errors

# The wheel velocity commands last sent (rad/s).
float32[4] command
