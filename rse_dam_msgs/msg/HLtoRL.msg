# Is there a new command we are sending through?
bool new_command

# Type of controller to use
# - 1 = follow controller
# - 2 = relative jacobian controller (master)
# - 3 = relative jacobian controller (slave)
int16 ctrl_type
#
# Cartesian space trajectory to follow
csa_msgs/TimedPoseArray poses
#
# Joint trajectory goal for the arm to follow
moveit_msgs/RobotTrajectory trajectory

