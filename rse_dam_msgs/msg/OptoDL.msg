# Type of move
#  - 1 = dual_arm
#  - 2 = left_arm
#  - 3 = right_arm
int16 move_type
#
# Pose of the target object
geometry_msgs/Pose object_start
#
# Desired final pose of the object
geometry_msgs/Pose object_goal
#
# Grasping pose of the left arm in the world frame
geometry_msgs/Pose left_grasp_pose
#
# left gripper close distance (meters)
float64 left_gap
#
# Grasping pose of the right arm in the world frame
geometry_msgs/Pose right_grasp_pose
#
# Right gripper close distance (meters)
float64 right_gap
#
# Speed at which to move the object (m/s)
float64 move_speed
#
# Pick and place lift height (meters)
float64 lift_height
