# Everything the demo supervisor (mecanum_demo) has decided. Published on /supervisor/state at
# 10 Hz and on every change, transient local. The panel and the TV draw themselves from this
# one message and keep no state of their own.

std_msgs/Header header

# idle, running, hold, gate, live, stowing, ended, stopping, stopped, manual, estop
string state
# "1".."5", "single" or "none"
string mode
string mode_label          # "Mode 1 · Show loop"

# The playlist being run and where it is.
string[] playlist          # feature keys
string[] playlist_labels   # their display names, same order
uint8 index                # current feature in playlist
string feature             # current feature key, "" when none
string step_label          # "Slide in sideways"
bool step_is_arm           # the current step is an arm placeholder
float32 eta_s              # time left in the playlist; -1 when unknown
bool eta_plus_teleop       # a teleop step of unknown length is still ahead
bool loop                  # the playlist restarts when it ends
uint32 loop_count          # 1 on the first pass

# Where the robot is and what it is doing.
string location            # named place key: home, stage, dock, ..., or "other"
string location_label      # "At home"
geometry_msgs/Pose2D pose  # in the map frame
bool pose_valid
bool base_moving
bool arm_stowed
bool arm_integrated        # false: every arm step is a placeholder

# Manual mode: "drive" or "arm"
string manual_sub
bool manual_blocked        # Manual driving is held because someone is close (checks on)
bool manual_checks         # Manual with the safety checks on (off on every entry to Manual:
                           # then the visitor hold, cliff sensors and bumpers don't stop the base)

# Hold: seconds the area has been clear (resumes at hold_clear_needed_s)
float32 hold_clear_s
float32 hold_clear_needed_s
bool person_close          # someone is close, whether or not the hold check acts on it
string hold_check          # on | off_loop (no holds until this loop ends) | off (until toggled)

# Teleop gate, live session and its result.
GateCheck[] checks
float32 gate_remaining_s   # until the automatic fallback; -1 when the gate waits forever
float32 live_elapsed_s
float32 live_idle_s        # counts up to live_idle_limit_s for the automatic end
float32 live_idle_limit_s
string ended_reason
float32 ended_length_s
uint32 demo_count          # demonstrations saved so far
uint32 last_demo           # number of the demonstration a replay uses

string message             # fault or stop reason
string stop_kind           # for stopped: "staff", "fault" or "estop"
bool can_resume
string notice              # non-blocking note, for example an arm placeholder running

FeatureStatus[] features
string[] allowed_actions
