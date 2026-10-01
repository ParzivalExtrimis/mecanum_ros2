# Commands for the ESP32 motor controller, one entry per motor.
#
# This is the stand-in wire format used between the driver's sim transport and the simulated
# ESP32 (mecanum_simulation/sim_motor_plant). The real ESP32 protocol is pending the firmware
# rewrite; the real transport will encode the same three fields in that protocol.

std_msgs/Header header

# Motor address on the ESP32 (STAND-IN mapping, see STANDINS.md).
uint8[] motor_id

# Spin direction: +1, -1, or 0 for stopped.
int8[] direction

# Velocity value in ESP32 units. STAND-IN: normalized 0.0 to 1.0 until the firmware defines
# its units and range.
float32[] value
