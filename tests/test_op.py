#!/usr/bin/env python

"""
  A simple operator command publisher with prebuilt inputs.
  
  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.
  
  TODO: IMPROVE DOCUMENTATION
"""


import sys
import rospy

from geometry_msgs.msg import Pose
from rse_dam_msgs.msg import OptoDL, DLtoOp


test = 6

# Horizontal move test
if (test==1):
    start_pose = Pose()
    start_pose.position.x = 0.209
    start_pose.position.y = −0.075
    start_pose.position.z =  0.24952
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 1.0

    goal_pose = Pose()
    goal_pose.position.x = 0.209
    goal_pose.position.y = 0.075
    goal_pose.position.z =  0.24952
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0

    # Not tested
    left_grasp_pose = Pose()
    right_grasp_pose = Pose()
    
    msg = OptoDL()
    msg.move_type = 1
    msg.template = 2
    msg.object_start = start_pose
    msg.object_goal = goal_pose
    msg.left_grasp_pose = left_grasp_pose
    msg.right_grasp_pose = right_grasp_pose
    msg.left_gap = 0.0
    msg.right_gap = 0.0
    msg.move_speed = 0.01
    msg.lift_height = 0.0
    
# Vertical move test
elif (test==2):
    start_pose = Pose()
    start_pose.position.x = 0.24
    start_pose.position.y = 0.0
    start_pose.position.z = 0.15
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 1.0

    goal_pose = Pose()
    goal_pose.position.x = 0.24
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.38
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0

    # Not tested
    left_grasp_pose = Pose()
    right_grasp_pose = Pose()
    
    msg = OptoDL()
    msg.move_type = 1
    msg.template = 2
    msg.object_start = start_pose
    msg.object_goal = goal_pose
    msg.left_grasp_pose = left_grasp_pose
    msg.right_grasp_pose = right_grasp_pose
    msg.left_gap = 0.0
    msg.right_gap = 0.0
    msg.move_speed = 0.01
    msg.lift_height = 0.0

# In-place rotation test
elif (test==3):
    start_pose = Pose()
    start_pose.position.x = 0.24
    start_pose.position.y = 0.0
    start_pose.position.z = 0.26035
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 1.0

    goal_pose = Pose()
    goal_pose.position.x = 0.24
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.26035
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.087
    goal_pose.orientation.w = 0.996

    # Not tested
    left_grasp_pose = Pose()
    right_grasp_pose = Pose()

    msg = OptoDL()
    msg.move_type = 1
    msg.template = 3
    msg.object_start = start_pose
    msg.object_goal = goal_pose
    msg.left_grasp_pose = left_grasp_pose
    msg.right_grasp_pose = right_grasp_pose
    msg.left_gap = 0.0
    msg.right_gap = 0.0
    msg.move_speed = 0.01
    msg.lift_height = 0.0

# Fail: out of reach
elif (test==4):
    start_pose = Pose()
    start_pose.position.x = 0.24
    start_pose.position.y = 0.0
    start_pose.position.z = 0.15
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 1.0

    goal_pose = Pose()
    goal_pose.position.x = 0.24
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.45
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0

    # Not tested
    left_grasp_pose = Pose()
    right_grasp_pose = Pose()

    msg = OptoDL()
    msg.move_type = 1
    msg.template = 2
    msg.object_start = start_pose
    msg.object_goal = goal_pose
    msg.left_grasp_pose = left_grasp_pose
    msg.right_grasp_pose = right_grasp_pose
    msg.left_gap = 0.0
    msg.right_gap = 0.0
    msg.move_speed = 0.01
    msg.lift_height = 0.0

# Fail: Bad orientation
elif (test==5):
    start_pose = Pose()
    start_pose.position.x = 0.24
    start_pose.position.y = 0.0
    start_pose.position.z = 0.15
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 1.0

    goal_pose = Pose()
    goal_pose.position.x = 0.24
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.38
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.087
    goal_pose.orientation.w = 0.996

    # Not tested
    left_grasp_pose = Pose()
    right_grasp_pose = Pose()

    msg = OptoDL()
    msg.move_type = 1
    msg.template = 2
    msg.object_start = start_pose
    msg.object_goal = goal_pose
    msg.left_grasp_pose = left_grasp_pose
    msg.right_grasp_pose = right_grasp_pose
    msg.left_gap = 0.0
    msg.right_gap = 0.0
    msg.move_speed = 0.01
    msg.lift_height = 0.0

# Fail: Too fast
elif (test==6):
    start_pose = Pose()
    start_pose.position.x = 0.24
    start_pose.position.y = 0.0
    start_pose.position.z = 0.15
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 1.0

    goal_pose = Pose()
    goal_pose.position.x = 0.24
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.38
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0

    # Not tested
    left_grasp_pose = Pose()
    right_grasp_pose = Pose()

    msg = OptoDL()
    msg.move_type = 1
    msg.template = 2
    msg.object_start = start_pose
    msg.object_goal = goal_pose
    msg.left_grasp_pose = left_grasp_pose
    msg.right_grasp_pose = right_grasp_pose
    msg.left_gap = 0.0
    msg.right_gap = 0.0
    msg.move_speed = 1.00
    msg.lift_height = 0.0

def pubber():
    pub = rospy.Publisher("operator/to_dl", OptoDL, queue_size=10)
    rospy.init_node("pubber", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
        print("published")
        
if __name__ == "__main__":
    try:
        pubber()
    except rospy.ROSInterruptException:
        pass
