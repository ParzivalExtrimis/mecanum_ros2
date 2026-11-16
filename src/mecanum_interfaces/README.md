# mecanum_interfaces

Message, service and action definitions for the mecanum base and the booth demo
([docs/demo_app.md](../../docs/demo_app.md)). The task and mission actions follow in Phase 7 of
[docs/plan.md](../../docs/plan.md).

| Message | Used on | Purpose |
|---|---|---|
| `SafetyState` | `/safety/state` | What the safety supervisor (`mecanum_safety`) blocks: the latched e-stop, whether all motion is stopped, whether the sensor blocks are bypassed (booth Manual), and one reason per active block. |
| `DriveState` | `/drive/state` | The drive ESP32's reports as the hardware plugin (`mecanum_hardware/Esp32System`) receives them, 100 Hz: wheel counts and pulses, faults. See `docs/hardware_drive_test.md`, "Protocol". |
| `SupervisorState` | `/supervisor/state` | The booth demo's whole state, as the panel and the TV draw it: state, mode, playlist, step, ETA, the teleop gate, the live session, the location and pose, feature readiness and `allowed_actions`, the visitor hold setting (`hold_check`) and Manual's safety checks (`manual_checks`). |
| `GateCheck` | in `SupervisorState` | One teleop gate check: name, ok, detail. |
| `FeatureStatus` | in `SupervisorState` | One feature: whether it can run from here, and why not. |
| `WheelCommands` | `/esp32_driver/wheel_commands` | Debug output of the drive node. It shows the clamped twist, the wheel speeds before and after saturation scaling, the calibrated motor commands, and whether the timeout fired. These are **commands, not feedback**. |

| Service | Used on | Purpose |
|---|---|---|
| `SupervisorCommand` | `/supervisor/command` | A request to the demo supervisor: action, args and request id; the reply accepts it or refuses it with a reason. The actions are listed in the file. |

| Action | Used on | Purpose |
|---|---|---|
| `MoveSegment` | `/demo/move_segment` | One prewritten base move of the booth demo (move, turn or spin to a map-frame pose), run by `mecanum_demo/segment_driver`. |

Build:

```bash
colcon build --packages-select mecanum_interfaces
```
