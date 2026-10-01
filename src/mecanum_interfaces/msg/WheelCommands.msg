# Debug view of what the drive node actually sent, one entry per wheel.
# Published on <driver>/wheel_commands. This is NOT feedback and NOT odometry: the motors
# have no encoders, and every value here is a command.

std_msgs/Header header

# Wheel names in kinematics order: front_left, front_right, rear_left, rear_right.
string[] wheel_names

# Body twist after clamping to the velocity limits [m/s, m/s, rad/s].
float64 vx
float64 vy
float64 wz

# Wheel speed from inverse kinematics, before saturation scaling [rad/s].
float64[] omega_requested

# Wheel speed after proportional saturation scaling [rad/s].
float64[] omega_sent

# Common factor applied to all wheels by saturation scaling (1.0 = no scaling).
float64 saturation_factor

# What went to the transport, after per-wheel calibration.
uint8[] motor_id
int8[] direction
float32[] value

# True when no cmd_vel arrived within cmd_timeout and zeros were sent.
bool timed_out
