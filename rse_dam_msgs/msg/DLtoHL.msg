# DL status:
#  -
int16 status
#
# Type of move that is being performed
#  - 
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
# - 
int16 command
