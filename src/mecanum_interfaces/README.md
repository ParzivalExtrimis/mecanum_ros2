# mecanum_interfaces

Message definitions for the mecanum base. Safety interfaces are added in Phase 4 and the task
and mission actions in Phase 7 of [docs/plan.md](../../docs/plan.md).

| Message | Used on | Purpose |
|---|---|---|
| `WheelCommands` | `/esp32_driver/wheel_commands` | Debug output of the drive node. It shows the clamped twist, the wheel speeds before and after saturation scaling, the calibrated motor commands, and whether the timeout fired. These are **commands, not feedback**. |

Build:

```bash
colcon build --packages-select mecanum_interfaces
```
