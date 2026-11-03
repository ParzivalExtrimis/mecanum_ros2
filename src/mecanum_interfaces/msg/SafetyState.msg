# What the safety supervisor (mecanum_safety) is blocking. Published on /safety/state at 10 Hz
# and on every change.

std_msgs/Header header

# The e-stop has latched; only the supervisor's ~/reset service clears it.
bool estop_latched

# All motion is blocked: the e-stop, or a lost sensor that stops the robot.
bool stopped

# One entry per active block, sorted, for example "estop", "cliff front_left",
# "bumper front", "lost /scan". Directional blocks allow only motion away from their side.
string[] reasons
