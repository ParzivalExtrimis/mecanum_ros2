# mecanum_interfaces

Message definitions for the mecanum base. The task and mission actions follow in Phase 7 of
[docs/plan.md](../../docs/plan.md).

| Message | Used on | Purpose |
|---|---|---|
| `SafetyState` | `/safety/state` | What the safety supervisor (`mecanum_safety`) blocks: the latched e-stop, whether all motion is stopped, and one reason per active block. |
| `WheelCommands` | `/esp32_driver/wheel_commands` | Debug output of the drive node. It shows the clamped twist, the wheel speeds before and after saturation scaling, the calibrated motor commands, and whether the timeout fired. These are **commands, not feedback**. |

Build:

```bash
colcon build --packages-select mecanum_interfaces
```
