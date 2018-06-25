# DL status:
# - 0 = Standby
# - 1 = Checking commands 
# - 2 = Planning
# - 3 = Plan returned, no errors
# - 4 = Planning failed
# - 5 = Plan executing, no errors
# - 6 = Execution finished successfully
# - 7 = Execution failed
int16 dl_status
#
# Right arm status
int16 right_status
#
# Left arm status
int16 left_status
#
# Failure information
string fail_info
