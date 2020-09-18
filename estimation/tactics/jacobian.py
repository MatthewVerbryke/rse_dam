#!/usr/bin/env python

"""
  A solver which can solve for the both individual Jacobians and the 
  relative Jacobian of a dual-arm robot.
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test 
"""


import numpy as np
import rospy
import tf
from tf import transformations as trfms

import solution


class DualArmJacobianSolver(object):
    """
    The dual-arm jacobian solver object.
    """
    
    def __init__(self, robot, left_base, left_eef, right_base, right_eef):
        
        # Parameters
        self.robot = robot
        self.left_base = left_base
        self.left_eef = left_eef
        self.right_base = right_base
        self.right_eef = right_eef
        
        # Variables
        self.master = ""
        
        # Create a TF listener
        self.listener = tf.TransformListener()
        
    def get_relative_jacobian(self, J_A, J_B, R_21, R_24, p_23):
        """
        Get the Relative jacobian matrix of the dual-arm setup. Which 
        arm is currently the "master" arm is not handled inside this
        function.
        
        Parameters
        ----------
        J_A : Arm A Jacobian Matrix
        J_B : Arm B Jacobian Matrix
        R_21 : Arm A EEF to Arm A base rotation matrix (Arm A EFF frame)
        R_24 : Arm A EEF to Arm B base rotation matrix (Arm A EFF frame)
        p_23 : Arm A EFF to Arm B EFF pose difference (Arm A EFF frame)
        
        Returns
        -------
        J_R: Relative Jacobian Matrix
        """
        
        # Preliminaries
        p_23_skew_sym = np.array([0., -p_23[2], p_23[1]],
                                [p_23[0], 0., -p_23[0]],
                                [-p_23[1], p_23[0], 0.])
                                
        psi_23 = np.array([np.identity(3), -p_23_skew_sym],
                         [np.zeros(3), np.identity(3)])
        
        omega21 = np.array([R_21, np.zeros(3)],
                           [np.zeros(3), R_21])
        
        omega24 = np.array([R_24, np.zeros(3)],
                           [np.zeros(3), R_24])
                           
        # Calculate the relative Jacobian
        psi_23_omega_21 = np.multiply(psi_23, omega_21)
        J_R = np.array(-np.multiply(psi_23_omega_21, J_A), np.multiply(omega_24, J_B))
        
        return J_R
        
    def lookup_frame_transform(self, origin, target):       
        """
        Get the transform (translation and quaternion rotation) between 
        two frames using tf.
        """   
        
        # Try to get the info on the frames from tf if given a link name
        try:
            trans, rot = self.listener.lookupTransform(target, origin, rospy.Time(0))
            return [trans, rot]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to recieve the transform for {} to {}".format(origin, target))
            return [None, None]
        
    def update(self, q_left, q_right):
        """
        Update the function when called from the main script.
        """
        
        # Get the individual arm Jacobians
        J_left = solution.boxbot(q_left)
        J_right = solution.boxbot(q_right)
            
        # Determine relavant frame transformations
        if master = "left":
            J_A = J_left
            J_B = J_right
            trans_21 = self.lookup_frame_transform(self.left_eef, self.left_base)
            trans_24 = self.lookup_frame_transform(self.left_eef, self.right_base)
            trans_23 = self.lookup_frame_transform(self.left_eef, self.right_eef)
        elif master = "right":
            J_A = J_right
            J_B = J_left
            trans_21 = self.lookup_frame_transform(self.right_eef, self.right_base)
            trans_24 = self.lookup_frame_transform(self.right_eef, self.left_base)
            trans_23 = self.lookup_frame_transform(self.right_eef, self.left_eef)
        else:
            return [J_left, J_right, None], "ok"
            
        # Determine the relative Jacobian
        R_21 = trfms.quaternion_matrix(trans_21[1])
        R_24 = trfms.quaternion_matrix(trans_24[1])
        p_23 = trans_23[0]
        J_R = self.get_relative_jacobian(J_left, J_right, R_21, R_24, p_23)

        return [J_left, J_right, J_R], "ok"
        
