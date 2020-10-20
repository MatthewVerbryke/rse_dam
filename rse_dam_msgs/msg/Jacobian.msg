# Since the Jacobian matrix for motion in 6-DOF space should always have 6-rows,
# we are able to pass it through in ROS msgs row-by-row, regardless of the number 
# of DOFs in the manipulator/robot
#
# Descriptive name for the jacobian
string name
#
# Base frame of reference for the jacobian
string frame
#
# the actual rows of the matrix
float64[] row0
float64[] row1
float64[] row2
float64[] row3
float64[] row4
float64[] row5
