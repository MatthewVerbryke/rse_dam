# Type of move
#  - 1 = bimanual
#  - 2 = coordinated
#  - 3 = uncoordinated
std_msgs/Int16 move_type
#
# Pose of the target object
geometry_msgs/Pose object_start
#
# Desired final pose of the object
geometry_msgs/Pose object_goal
#
# Grasping pose of the left arm
geometry_msgs/Pose left_grasp_pose
#
# left gripper close distance (meters)
std_msgs/Float64 left_gap
#
# Grasping pose of the right arm
geometry_msgs/Pose right_grasp_pose
#
# Right gripper close distance (meters)
std_msgs/Float64 right_gap
#
# Speed at which to move the object (m/s)
std_msgs/Float64 move_speed
#
# Pick and place lift height (meters)
std_msgs/Float64 lift_height
