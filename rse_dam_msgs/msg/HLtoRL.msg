# Type of controller to use
# - 1 = follow controller
# - 2 = relative jacobian controller (master)
# - 3 = relative jacobian controller (slave)
int16 ctrl_type
#
# Cartesian space trajectory to follow
geometry_msgs/PoseArray poses
#
# Joint trajectory goal for the arm to follow
moveit_msgs/RobotTrajectory trajectory

