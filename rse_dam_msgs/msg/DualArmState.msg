# A message containing the state data of a dual-arm robotic system
#
# Includes:
#  * End-effector states of each arm
#  * Joint states of each arm
#  * Jacobians of each arm
#  * Relative Jacobian
#  * State of a targeted object
#
# Curretly WIP

Header header

sensor_msgs/JointState left_joint_state
rse_dam_msgs/Jacobian left_jacobian
geometry_msgs/Pose left_eef_state
diagnostic_msgs/DiagnosticStatus[] left_status

sensor_msgs/JointState right_joint_state
rse_dam_msgs/Jacobian right_jacobian
geometry_msgs/Pose right_eef_state
diagnostic_msgs/DiagnosticStatus[] right_status

rse_dam_msgs/Jacobian rel_jacobian


