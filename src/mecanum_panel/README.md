# mecanum_panel

The booth demo's web pages, and the small server that serves them. The pages are plain ES
modules with no build step, no Node.js and no CDN. They talk to ROS only through the
rosbridge websocket that `mecanum_demo/demo.launch.py` starts. The design is in
[docs/Robot control panel app architecture.md](../../docs/Robot%20control%20panel%20app%20architecture.md);
the way it was built is in [docs/demo_app.md](../../docs/demo_app.md).

## Files

| File | Purpose |
|---|---|
| `mecanum_panel/panel_server.py` | Static file server for `web/` (port `port`, default 8080). It generates `/config.js` with the bridge port and whether to show the simulation controls. |
| `web/common/rosbridge.js` | The ROS adapter: rosbridge v2 protocol (subscribe, advertise, publish, call_service) and reconnects. It never interprets a message. |
| `web/panel/` | The 800 × 480 touchscreen panel. Its modules follow the layers of the architecture doc: `state_store.js`, `router.js`, `command_client.js`, `drive_streamer.js`, `heartbeat.js`, plus `screens/` for the eight screens and four overlays. |
| `web/tv/` | The booth TV, a 1280 × 720 canvas. It shows the counter, the status tag, the playlist, the booth map with the robot, its lidar and the path ahead, a live lidar view, and the caption. The wrist-camera panel is a placeholder. |
| `web/sim/` | Simulation-only controls: the visitor, the e-stop, and the arm-checks development switch. |
| `web/index.html` | The development dashboard: the panel at actual size, the TV and the simulation controls, side by side. |

## Pages

- `http://<host>:8080/panel/`: the panel. `?fit=0` shows it at 1:1 instead of scaled to the window, and `?ws=ws://robot:9090` points it at another bridge.
- `http://<host>:8080/tv/`: the TV.
- `http://<host>:8080/`: the dashboard. The simulation card shows only when `sim_controls` is true.
- `http://<host>:8080/panel/?fixture=<name>`: the panel drawing one sample state from `web/panel/fixtures.js`, with no robot and no websocket. Names: `home`, `running`, `replay`, `hold`, `hold_off`, `gate`, `live`, `ended`, `stopped`, `manual`, `manual_checks_on`, `manual_arm`, `estop`, `health`, `linklost`. For development only.

## Rules the panel keeps

- **The robot decides.** Every screen is drawn from the latest `/supervisor/state`. A button works only if its action is in `allowed_actions`.
- **One command at a time.** A second tap while a command is pending is ignored, except STOP, which always goes out at once.
- **Link lost.** If no state has arrived for more than 1 s, a full-screen notice covers everything.
- **Heartbeat.** `/panel/heartbeat` is sent at 2 Hz. `/panel/drive` is sent at 20 Hz while a D-pad key is held.

## Run

```bash
ros2 launch mecanum_demo demo.launch.py            # with the robot's bringup running
ros2 launch mecanum_bringup demo_sim.launch.py     # everything, in Gazebo
```

## Tests

`colcon test --packages-select mecanum_panel` runs the copyright, flake8 and pep257 checks on
the server. The pages are checked by hand and with headless Firefox screenshots
(docs/demo_app.md).
