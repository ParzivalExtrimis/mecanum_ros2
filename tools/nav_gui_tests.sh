#!/bin/bash
# Guided GUI tests of the navigation stack (docs/plan.md Phase 6), in Gazebo and RViz.
#
#   tools/nav_gui_tests.sh            open the three terminals below
#   tools/nav_gui_tests.sh stop       stop everything the tests started (also a stray gz sim)
#
# Terminal 1 "nav: launch"   mecanum_bringup navigation.launch.py, Gazebo GUI and RViz, both on
#                            the NVIDIA GPU when there is one (NAV_GUI_GPU=default: not)
# Terminal 2 "nav: steps"    the tests, one at a time: Enter runs a step, s skips it, q quits
# Terminal 3 "nav: monitor"  the collision monitor's action and the safety gate's blocks, on change
#
# Goal timeouts are doubled for the slower simulation with the windows open
# (NAV_GUI_TIME_SCALE=1 for the headless values).
#
# Each step says what it does, what to expect and what to watch. Goals go to Nav2 with
# `ros2 action send_goal`; test objects are put into Gazebo with its create service, as
# tools/nav_check.py does. After each step the robot's ground-truth pose is printed. Contact is
# not judged here: watch Gazebo (tools/nav_check.py judges it headless).
#
# Coordinates: goals are in the map frame, whose origin is the spawn pose, world (0, -1, 0).
# Test objects are in world coordinates. map = world + (0.007, 1.028), the map's best fit.

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
SELF="$HERE/$(basename "${BASH_SOURCE[0]}")"
WORLD=restroom

ros_env() {
    # The terminals are not login shells and do not read ~/.bashrc: the middleware is set here.
    export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
    set +u  # the ROS setup files use unset variables
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    # shellcheck disable=SC1091
    source "$REPO/install/setup.bash"
    set -u
    cd "$REPO" || exit 1
}

# --- terminals ------------------------------------------------------------------------------

open_terminals() {
    if ! command -v gnome-terminal > /dev/null; then
        echo "gnome-terminal not found. Run these yourself, one per terminal:"
        echo "  $SELF launch"; echo "  $SELF steps"; echo "  $SELF monitor"
        exit 1
    fi
    if [ ! -f "$REPO/install/setup.bash" ]; then
        echo "No install/ in $REPO: run colcon build first."; exit 1
    fi
    if pgrep -f "^gz sim" > /dev/null || pgrep -f "bt_navigator" > /dev/null; then
        echo "A simulation or Nav2 is already running. Stop it first: $SELF stop"; exit 1
    fi
    # gnome-terminal does not pass this shell's environment on: hand the ROS variables over.
    local env_vars="RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
    local v
    for v in ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS CYCLONEDDS_URI \
            NAV_GUI_GPU NAV_GUI_TIME_SCALE; do
        [ -n "${!v:-}" ] && env_vars="$env_vars $v=${!v}"
    done
    local t
    for t in launch monitor steps; do
        gnome-terminal --title="nav: $t" -- env $env_vars bash -c "'$SELF' $t; exec bash"
        [ "$t" = launch ] && sleep 2
    done
}

run_launch() {
    ros_env
    # On a PRIME laptop the windows otherwise render on the integrated GPU, beside the sim's
    # GPU sensors. NAV_GUI_GPU=default keeps the default GPU.
    if [ "${NAV_GUI_GPU:-nvidia}" = nvidia ] && nvidia-smi -L > /dev/null 2>&1; then
        export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
        local egl=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
        [ -f "$egl" ] && export __EGL_VENDOR_LIBRARY_FILENAMES=$egl
        echo "Gazebo and RViz render on the NVIDIA GPU (NAV_GUI_GPU=default for the default one)."
    fi
    echo "Starting Gazebo, the robot, AMCL, Nav2 and RViz. Ctrl-C here stops them;"
    echo "if Gazebo stays open afterwards: $SELF stop"
    ros2 launch mecanum_bringup navigation.launch.py
}

wait_for_nav2() {
    echo -n "Waiting for Nav2 to be active"
    until ros2 lifecycle get /bt_navigator 2> /dev/null | grep -q active \
            && ros2 lifecycle get /amcl 2> /dev/null | grep -q active; do
        echo -n "."; sleep 2
    done
    echo " ready."
}

run_monitor() {
    ros_env
    wait_for_nav2
    echo "The collision monitor's action and the safety gate's blocks, printed when they change"
    echo "(after lasting 0.5 s; stops at once):"
    python3 -u - << 'EOF2'
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.msg import CollisionMonitorState
from mecanum_interfaces.msg import SafetyState

ACTIONS = {0: 'none', 1: 'stop', 2: 'slowdown', 3: 'approach', 4: 'limit'}
HOLD = 0.5  # s a new state must last before it is printed (a stop is printed at once)
rclpy.init()
node = rclpy.create_node('gui_tests_monitor')
start = time.monotonic()
shown, pending = {}, {}


def offer(key, text, urgent=False):
    if pending.get(key, (None,))[0] != text:
        pending[key] = (text, time.monotonic())
    if urgent:
        flush()


def flush():
    now = time.monotonic()
    if now - start < 3.0:  # the gate reports every sensor lost until it has heard from it
        return
    for key, (text, since) in pending.items():
        urgent = text.startswith(('stop', 'STOPPED'))
        if shown.get(key) != text and (urgent or now - since >= HOLD):
            shown[key] = text
            print(f'{time.strftime("%H:%M:%S")}  {key:18s} {text}')


def on_cm(m):
    action = ACTIONS.get(m.action_type, m.action_type)
    offer('collision monitor', f'{action} {m.polygon_name}'.strip(), action == 'stop')


def on_safety(m):
    blocks = ', '.join(m.reasons) or 'clear'
    offer('safety gate', ('STOPPED: ' if m.stopped else '') + blocks, m.stopped)


node.create_subscription(CollisionMonitorState, '/collision_monitor_state', on_cm,
                         QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE))
node.create_subscription(SafetyState, '/safety/state', on_safety, 10)
node.create_timer(0.1, flush)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
EOF2
}

stop_all() {
    pkill -INT -f "ros2 launch mecanum_bringup navigation.launch.py"
    sleep 10
    # gz sim often survives the launch's SIGINT; a TERM by pid ends it, sometimes only after
    # several seconds.
    for p in $(pgrep -f "^gz sim"); do kill -TERM "$p"; done
    pkill -TERM -x rviz2
    local i
    for i in $(seq 15); do
        pgrep -f "^gz sim" > /dev/null || break
        sleep 1
    done
    pgrep -af "^gz sim|bt_navigator|controller_server|nav2_amcl" || echo "stopped"
}

# --- helpers for the steps ------------------------------------------------------------------

# goal <label> <x> <y> <heading: E|N|W|S> <timeout s>: NavigateToPose in the map frame. The
# timeout is in sim seconds at full speed; with the windows open the simulation runs at about
# 0.7 of real time, so it is stretched by NAV_GUI_TIME_SCALE (default 2).
goal() {
    local label=$1 x=$2 y=$3 h=$4 z w
    local t=$(( $5 * ${NAV_GUI_TIME_SCALE:-2} ))
    case $h in
        E) z=0; w=1 ;;
        N) z=0.7071068; w=0.7071068 ;;
        W) z=1; w=0 ;;
        S) z=-0.7071068; w=0.7071068 ;;
    esac
    echo ">>> Nav2 goal '$label': map ($x, $y), facing $h (cancelled after $t s)"
    timeout -s INT "$t" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: $x, y: $y}, \
          orientation: {z: $z, w: $w}}}}" | grep -E "accepted|rejected|status|Canceling"
}

# box_sdf <name> <x> <y> <z centre> <sx> <sy> <sz> [hidden]: a static box, world coordinates.
# hidden: invisible to the lidars and the camera (as glass in sim, shortcut S4); the
# ultrasonics still hear it.
box_sdf() {
    local name=$1 x=$2 y=$3 z=$4 sx=$5 sy=$6 sz=$7 hidden=${8:-}
    local geometry="<geometry><box><size>$sx $sy $sz</size></box></geometry>"
    local extra='<material><ambient>0.8 0.3 0.1 1</ambient><diffuse>0.8 0.3 0.1 1</diffuse></material>'
    if [ -n "$hidden" ]; then
        extra="<transparency>0.7</transparency><visibility_flags>268435456</visibility_flags>$extra"
    fi
    echo "<sdf version=\\\"1.9\\\"><model name=\\\"$name\\\"><static>true</static><pose>$x $y $z 0 0 0</pose><link name=\\\"link\\\"><collision name=\\\"collision\\\">$geometry</collision><visual name=\\\"visual\\\">$geometry$extra</visual></link></model></sdf>"
}

insert() {
    echo ">>> Gazebo: insert $1"
    gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean --timeout 5000 --req "sdf: \"$(box_sdf "$@")\"" > /dev/null
}

remove() {
    echo ">>> Gazebo: remove $1"
    gz service -s "/world/$WORLD/remove" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
        --timeout 5000 --req "name: \"$1\" type: MODEL" > /dev/null
}

# The robot's ground-truth pose (Gazebo), in the world and in the map frame.
# truth raw: print only "x y" in the world, for placing a box relative to the robot.
truth() {
    python3 - "${1:-}" << 'EOF'
import math, sys, rclpy
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node('gui_tests_truth'); got = []
n.create_subscription(Odometry, '/sim/ground_truth/odom', got.append, 10)
for _ in range(50):
    if got: break
    rclpy.spin_once(n, timeout_sec=0.1)
if got:
    p = got[-1].pose.pose; q = p.orientation
    if sys.argv[1] == 'raw':
        print(f'{p.position.x:.3f} {p.position.y:.3f}'); sys.exit(0)
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    print(f'    ground truth: world ({p.position.x:.2f}, {p.position.y:.2f}), '
          f'map ({p.position.x + 0.007:.2f}, {p.position.y + 1.028:.2f}), heading {yaw:.0f} deg')
else:
    print('    ground truth: no message')
EOF
}

# step <title> <what you should see> <what to watch>; returns 1 to skip.
step() {
    echo
    echo "=================================================================================="
    echo "STEP: $1"
    echo "  Expect: $2"
    echo "  Watch:  $3"
    local answer
    read -r -p "  [Enter] run, [s] skip, [q] quit: " answer
    case $answer in
        q|Q) echo "Done. Stop everything with: $SELF stop"; exit 0 ;;
        s|S) return 1 ;;
    esac
    return 0
}

home() { goal home 0.0 0.0 E 150; truth; }

# --- the steps ------------------------------------------------------------------------------

run_steps() {
    ros_env
    wait_for_nav2
    sleep 3
    echo "In RViz: the map, the particle cloud on the robot at home, both costmaps, the keepout"
    echo "zones (the glass screen in the lobby, the stall partitions, the threshold at the"
    echo "corridor's east end). Gazebo: the robot in the corridor, facing east."
    truth

    if step "Through the doorway into the restroom" \
        "the robot drives east, turns to face north BEFORE the doorway, drives through it and stops inside facing north (30 to 40 s)" \
        "the turn happens in the corridor, not in the doorway; no scraping of the jambs in Gazebo; a short pause in the doorway is normal"; then
        goal doorway_in 3.45 2.53 N 120; truth
    fi

    if step "Strafe 1.35 m to the right, same heading" \
        "the robot slides sideways (east) while still facing north; it turns at most about 15 deg (6 s)" \
        "the wheels in Gazebo: a sideways slide, not a turn-drive-turn"; then
        goal strafe_right 4.80 2.53 N 60; truth
    fi

    if step "To the lobby, then behind the glass screen" \
        "out through the doorway, west along the corridor into the lobby (about 60 s); then into the pocket behind the glass, going AROUND its end, never through it (about 100 s)" \
        "RViz: the global path bends around the keepout zone over the glass; the robot creeps and turns slowly in the pocket behind it (normal, see plan Phase 6)"; then
        goal lobby -0.29 2.53 N 150; truth
        goal behind_glass 0.21 4.43 E 180; truth
    fi

    if step "Back home" "the robot returns to its start, facing east" "nothing in particular"; then
        home
    fi

    if step "Past the threshold (must be refused)" \
        "Nav2 rejects or aborts the goal (status ABORTED); the robot does not cross the threshold at x 5.35 in the world" \
        "RViz: the threshold's keepout zone closes the corridor's east end"; then
        goal past_threshold 6.31 0.03 E 60; truth
    fi

    if step "A box on the corridor route (go around)" \
        "a 0.4 m box at world (2.5, -1.35) appears in Gazebo and in the costmaps; the robot drives around it on the north side and reaches the corridor's east end (about 20 s)" \
        "about 20 cm of clearance; the D435i voxels on the box in RViz"; then
        home
        insert nc_box 2.5 -1.35 0.25 0.4 0.4 0.5
        sleep 1; goal box 4.51 0.02 E 180; truth
        remove nc_box
    fi

    if step "A low box, 0.10 m high, under the 2D lidar's plane (go around)" \
        "the box is not in the lidar scan; the D435i puts it in the voxel layer, and the robot goes around it as for the box" \
        "RViz: the red laser scan passes over the box, only the voxels mark it"; then
        home
        insert nc_low_box 2.5 -1.45 0.05 0.4 0.6 0.10
        sleep 1; goal low_box 4.51 0.02 E 180; truth
        remove nc_low_box
    fi

    if step "A box dropped 0.9 m ahead of the moving robot" \
        "the robot starts east; after 5 s a box appears on its route, about 1 to 1.5 m ahead of it; it slows, steps around it and reaches the end (about 35 s)" \
        "the collision monitor terminal: slowdown, maybe a stop; no contact"; then
        home
        ( sleep 5; insert nc_dropped 2.4 -1.35 0.25 0.4 0.4 0.5 ) &
        goal dropped_ahead 4.51 0.02 E 180; wait; truth
        remove nc_dropped
    fi

    if step "Unknown glass across the corridor (decision D6-a: stop before it)" \
        "a transparent panel at world x 2.6 that the lidars and the camera cannot see; the front ultrasonics hear it and the robot stops 10 to 20 cm short; Nav2 keeps retrying until the goal is cancelled" \
        "the collision monitor terminal: stop FrontStop; no contact with the panel"; then
        home
        insert nc_glass 2.6 -1.0 0.5 0.02 1.9 1.0 hidden
        sleep 1; goal unknown_glass 4.51 0.02 E 90; truth
        remove nc_glass
    fi

    if step "Reversing toward a box behind (rear ultrasonics)" \
        "the robot moves 0.6 m east of home; a box appears 0.25 m behind it; Nav2's BackUp (0.6 m) stops 15 to 25 cm short of the box and reports ABORTED (expected)" \
        "the robot must not touch the box; BackUp's own check usually stops it first, else the collision monitor terminal shows stop RearStop"; then
        goal reverse_start 0.61 0.03 E 60
        local xy bx
        read -r -a xy <<< "$(truth raw)"
        # The box's front face 0.25 m behind the robot's rear (0.405 m behind its centre).
        bx=$(python3 -c "print(f'{${xy[0]} - 0.405 - 0.25 - 0.1:.3f}')")
        insert nc_rear "$bx" "${xy[1]}" 0.3 0.2 0.7 0.6
        sleep 1.5
        echo ">>> Nav2 BackUp 0.6 m at 0.1 m/s"
        ros2 action send_goal /backup nav2_msgs/action/BackUp \
            "{target: {x: 0.6}, speed: 0.1, time_allowance: {sec: 20}}" | grep -E "accepted|status"
        truth
        read -r -a xy <<< "$(truth raw)"
        python3 -c "print(f'    gap between the robot and the box: {100 * (${xy[0]} - 0.405 - ($bx + 0.1)):.1f} cm')"
        remove nc_rear
    fi

    if step "Relocalization from a wrong start pose" \
        "AMCL is told the robot is 0.4 m and 25 deg away from where it is; the particles in RViz jump there, then the robot drives to the corridor's east end and the particles come back onto it within about a metre" \
        "RViz: the laser scan first misaligned with the map, then aligned again; the goal is still reached"; then
        home
        echo ">>> /initialpose: map (0.35, 0.20), heading 25 deg (truth: about (0, 0), 0 deg)"
        ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
            "{header: {frame_id: map}, pose: {pose: {position: {x: 0.35, y: 0.20}, \
              orientation: {z: 0.2164, w: 0.9763}}, covariance: [0.0625, 0, 0, 0, 0, 0, \
              0, 0.0625, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
              0, 0, 0, 0, 0, 0.0685]}}" > /dev/null
        sleep 2
        goal relocalize 4.51 0.02 E 120; truth
    fi

    if step "Back home" "the robot returns to its start" "nothing in particular"; then
        home
    fi

    echo
    echo "All steps done. Send your own goals with RViz's 'Nav2 Goal' tool if you like."
    echo "Stop everything: Ctrl-C in the launch terminal, then: $SELF stop"
}

case ${1:-} in
    "") open_terminals ;;
    launch) run_launch ;;
    steps) run_steps ;;
    monitor) run_monitor ;;
    stop) stop_all ;;
    *) echo "usage: $0 [launch|steps|monitor|stop]"; exit 1 ;;
esac
