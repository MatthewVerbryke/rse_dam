# Type of move that is being performed in relation to the arm
#  - 0 = gripper move
#  - 1 = dual-arm
#  - 2 = single-arm
int16 move_type
#
# Waypoints for the arm to follow during bimanual move
geometry_msgs/PoseArray poses
#
# Timestamps for the above waypoints
string stamps
#
# Pose target for an independent move
geometry_msgs/Pose target_pose
#
# DL command based on mediator return
# - 0 = Don't execute plan
# - 1 = Execute plan
int16 command
