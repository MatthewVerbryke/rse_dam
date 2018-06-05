# DL status:
#  -
std_msgs/Int16 status
#
# Type of move that is being performed
#  - 
std_msgs/Int16 move_type
#
# Waypoints for the arm to follow during bimanual move
geometry_msgs/PoseArray poses
#
# Timestamps for the above waypoints
std_msgs/String stamps
#
# Pose target for an independent move
geometry_msgs/Pose target_pose
#
# DL command based on mediator return
# - 
std_msgs/Int16 command
