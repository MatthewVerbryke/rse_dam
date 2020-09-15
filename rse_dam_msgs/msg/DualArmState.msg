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

string[] left_names
float64[] left_jacobian
geometry_msgs/Pose left_eef_state
diagnostic_msgs/DiagnosticStatus[] left_status

string[] right_names
float64[] right_jacobian
geometry_msgs/Pose right_eef_state
diagnostic_msgs/DiagnosticStatus[] right_status

float64[] rel_jacobian

geometry_msgs/Pose target_state


