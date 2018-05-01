#!/usr/bin/env python

"""
  Function(s) for determining the reachability of poses with the WidowX 
  robotic arm.

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
"""


from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
import tf


# Reference points
ref_point_1 = [0, 0.087, 0.2659] # arm_base_link   
ref_point_2 = [0, 0.152, 0.2659] # shoulder_link


def is_pose_reachable(pose):
    """
    Determine if the given pose is a reachable pose for the end effector
    of a 5 DOF robot arm similar in kinematic structure to the WidowX arm.
    """
    
    # Format the pose goal
    pose_goal = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    
    # Get 1st vector
    vector_1 = [(ref_point_2[0] - ref_point_1[0]),
                (ref_point_2[1] - ref_point_1[1]),
                (ref_point_2[2] - ref_point_1[2])]
    
    # Get 2nd vector
    vector_2 = [(pose_goal[0] - ref_point_1[0]),
                (pose_goal[1] - ref_point_1[1]),
                (pose_goal[2] - ref_point_1[2])]
    
    # Determine normal
    normal = np.cross(vector_1, vector_2)
    
    # Convert the quarternion given into a rotation matrix for easy rotation
    quat = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    hom_rot_matrix = tf.transformations.quaternion_matrix(quat)
    rot_matrix = hom_rot_matrix[0:3,0:3]

    # Get end effector orientaion in vector format
    unit_y = [0, 0, 1] # z is 'forward out' on the eef of the widowx
    eef_vector = np.dot(rot_matrix, unit_y)
    
    # Check if pose is within plane of arm
    result = np.dot(normal, eef_vector)
    
    if (result < 0.000001): # chalk it up to rounding errors
        return True
    else:
        rospy.logerr("End effector pose is not reachable.")
        return False
