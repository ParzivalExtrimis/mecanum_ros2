# mecanum_interfaces

Message definitions for the mecanum base. Action definitions for the task and mission
executive are added in phase 8.

| Message | Used on | Purpose |
|---|---|---|
| `MotorCommands` | `/esp32/motor_commands` | Per-motor `motor_id`, `direction` and `value`. This is the stand-in ESP32 wire format between the driver's `sim` transport and the simulated ESP32. The real transport will carry the same fields in the firmware's protocol. |
| `WheelCommands` | `/esp32_driver/wheel_commands` | Debug output of the drive node. It shows the clamped twist, the wheel speeds before and after saturation scaling, the calibrated motor commands, and whether the timeout fired. These are **commands, not feedback**. |

Build:

```bash
colcon build --packages-select mecanum_interfaces
```
