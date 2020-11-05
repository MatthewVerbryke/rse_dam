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


test = 2

# Horizontal move test
if (test==1):
    rel_pose = Pose()
    rel_pose.position.x = 0.0
    rel_pose.position.y = 0.0
    rel_pose.position.z = 0.2
    rel_pose.orientation.x = 1.0
    rel_pose.orientation.y = 0.0
    rel_pose.orientation.z = 0.0
    rel_pose.orientation.w = 0.0

    goal_pose = Pose()
    goal_pose.position.x = 0.209
    goal_pose.position.y = 0.175
    goal_pose.position.z = 0.24952
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = -0.707107
    goal_pose.orientation.z = 0.707107
    goal_pose.orientation.w = 0.0

    # Not tested
    left_grasp_pose = Pose()
    right_grasp_pose = Pose()
    
    msg = OptoDL()
    msg.move_type = 4
    msg.template = 0
    msg.object_start = Pose()
    msg.object_goal = goal_pose
    msg.left_grasp_pose = Pose()
    msg.right_grasp_pose = rel_pose
    msg.left_gap = 0.0
    msg.right_gap = 0.0
    msg.move_speed = 0.01
    msg.lift_height = 0.0
    
# Vertical move test
elif (test==2):
    rel_pose = Pose()
    rel_pose.position.x = 0.0
    rel_pose.position.y = 0.0
    rel_pose.position.z = 0.2
    rel_pose.orientation.x = 1.0
    rel_pose.orientation.y = 0.0
    rel_pose.orientation.z = 0.0
    rel_pose.orientation.w = 0.0

    goal_pose = Pose()
    goal_pose.position.x = 0.24
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.35
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0

    # Not tested
    left_grasp_pose = Pose()
    right_grasp_pose = Pose()
    
    msg = OptoDL()
    msg.move_type = 4
    msg.template = 0
    msg.object_start = Pose()
    msg.object_goal = goal_pose
    msg.left_grasp_pose = Pose()
    msg.right_grasp_pose = rel_pose
    msg.left_gap = 0.0
    msg.right_gap = 0.0
    msg.move_speed = 0.01
    msg.lift_height = 0.0

# In-place rotation test
elif (test==3):
    rel_pose = Pose()
    rel_pose.position.x = 0.0
    rel_pose.position.y = 0.0
    rel_pose.position.z = 0.2
    rel_pose.orientation.x = 1.0
    rel_pose.orientation.y = 0.0
    rel_pose.orientation.z = 0.0
    rel_pose.orientation.w = 0.0

    goal_pose = Pose()
    goal_pose.position.x = 0.209
    goal_pose.position.y = 0.1
    goal_pose.position.z = 0.24952
    goal_pose.orientation.x = 0.643
    goal_pose.orientation.y = -0.766
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 0.0

    # Not tested
    left_grasp_pose = Pose()
    right_grasp_pose = Pose()

    msg = OptoDL()
    msg.move_type = 4
    msg.template = 0
    msg.object_start = Pose()
    msg.object_goal = goal_pose
    msg.left_grasp_pose = Pose()
    msg.right_grasp_pose = rel_pose
    msg.left_gap = 0.0
    msg.right_gap = 0.0
    msg.move_speed = 0.01
    msg.lift_height = 0.0

def pubber():
    pub = rospy.Publisher("OptoDL", OptoDL, queue_size=10)
    rospy.init_node("pubber", anonymous=True)
    rate = rospy.Rate(10)
    i=1
    while i < 6:
        pub.publish(msg)
        rate.sleep()
        i += 1
        
if __name__ == "__main__":
    try:
        pubber()
    except rospy.ROSInterruptException:
        pass
