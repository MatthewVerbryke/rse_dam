#!/usr/bin/env python

"""
  IK solution correction for IKFast with the WidowX arm, specifically the 
  last joint (joint_5).

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentations
"""


from geometry_msgs.msg import Pose
import math
import numpy as np
import tf 


def correct_wrist_angle(eef_pose, goal_pose):
    """
    Add a correct wrist roll angle to match the goal pose for the WidowX
    arm. This is due to 'translationdirection5d' not considering roll angle
    about the end-effector pointing direction.
    
    WARNING HACK: this is a hacky solution that I know works for some 
    limited scenarios I intend to test. This is very likely a temporary 
    solution, and I'm not going to exhaustively test it.
    
    NOTE: assumes that the returned eef_pose is pointed in the right 
    direction, but is not rolled correctly

    """
       
    # Get quaternion components
    eef_x = eef_pose.orientation.x
    eef_y = eef_pose.orientation.y
    eef_z = eef_pose.orientation.z
    eef_w = eef_pose.orientation.w
    goal_x = goal_pose.orientation.x
    goal_y = goal_pose.orientation.y
    goal_z = goal_pose.orientation.z
    goal_w = goal_pose.orientation.w

    # Check same direction assumption
    eef_tf_matrix = tf.transformations.quaternion_matrix([eef_x, eef_y, eef_z, eef_w])
    goal_tf_matrix = tf.transformations.quaternion_matrix([goal_x, goal_y, goal_z, goal_w])
    eef_rot = eef_tf_matrix[0:3,0:3]
    goal_rot = goal_tf_matrix[0:3,0:3]
    
    # Determine unit vectors
    eef_xvec = np.dot(eef_rot, [1, 0, 0])
    eef_zvec = np.dot(eef_rot, [0, 0, 1])
    goal_xvec = np.dot(goal_rot, [1, 0, 0])
    goal_zvec = np.dot(goal_rot, [0, 0, 1])
    
    # Angle difference between the z-unit vectors
    z_diff = math.acos(np.dot(eef_zvec, goal_zvec)/1)
    
    #If z-unit-vectors match up, determine the angle between the two x-unit-vectors and its sign
    if (z_diff<=0.01): #tolerances are loose to compensate for relatively loose IK solver tolerances (~0.01 rad)
        theta = math.acos(np.dot(eef_xvec, goal_xvec)/1)
        cross_prod = np.cross(eef_xvec, goal_xvec)
        cross_unit = (cross_prod/np.linalg.norm(cross_prod))
        if np.allclose(cross_unit, eef_zvec, 1e-02, 0):
            theta_signed = theta
        else:
            theta_signed = -theta
        return theta_signed
    else:
        return False
