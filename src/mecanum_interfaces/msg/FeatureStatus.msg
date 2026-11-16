# Whether one single feature can be run now (mecanum_demo, /supervisor/state). The panel draws
# its feature chips from this; it never decides readiness itself.

string name      # key used in run_feature, for example "leave_home"
string label     # text for the chip, for example "Leave home"
bool ready       # its precondition holds at the robot's current place
string reason    # why not, when ready is false: "Needs to start at home"
bool loopable    # it ends where it starts, so it may be looped
bool arm         # it is an arm-only feature (a placeholder until the arm is integrated)
