#!/usr/bin/env python

"""
  A solver which can solve for the both individual Jacobians and the 
  relative Jacobian of a dual-arm robot.
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import numpy as np
import rospy
import tf
from tf import transformations as trfms

import solutions


class DualArmJacobianSolver(object):
    """
    The dual-arm Jacobian solver object.
    """
    
    def __init__(self, robot, arms, master):
        
        # Parameters
        self.robot = robot
        self.left_names = arms["left_arm"]["name"]
        self.left_index = arms["left_arm"]["index"]
        self.right_names = arms["right_arm"]["name"]
        self.right_index = arms["right_arm"]["index"]
        self.num_joints = len(self.left_names)
        self.left_base = "/" + arms["left_arm"]["base"]
        self.left_eef = "/" + arms["left_arm"]["eef"]
        self.right_base = "/" + arms["right_arm"]["base"]
        self.right_eef = "/" + arms["right_arm"]["eef"]
        
        # Variables
        self.master = master
        
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
        T_21 : Arm A EEF to Arm A base transformation matrix (Arm A EFF
               frame)
        T_24 : Arm A EEF to Arm B base transformation matrix (Arm A EFF
               frame)
        p_23 : Arm A EFF to Arm B EFF pose difference (Arm A EFF frame)
        
        Returns
        -------
        J_R: Relative Jacobian Matrix
        """
        
        # Build required matricies for equation
        p_23_skew_sym = np.array([[0., -p_23[2], p_23[1]],
                                 [p_23[2], 0., -p_23[0]],
                                 [-p_23[1], p_23[0], 0.]])
        
        psi_23 = np.identity(6)
        psi_23[0:3,3:6] = p_23_skew_sym
        
        omega_21 = np.zeros((6,6))
        omega_21[0:3,0:3] = R_21
        omega_21[3:6,3:6] = R_21
        
        omega_24 = np.zeros((6,6))
        omega_24[0:3,0:3] = R_24
        omega_24[3:6,3:6] = R_24
        
        # Calculate the relative Jacobian
        psi_23_omega_21 = np.matmul(psi_23, omega_21)
        J_R_left = -np.matmul(psi_23_omega_21, J_A)
        J_R_right = np.matmul(omega_24, J_B)
        J_R = np.hstack((J_R_left, J_R_right))
        
        return J_R
        
    def lookup_frame_transform(self, origin, target):       
        """
        Get the transform (translation and quaternion rotation) between 
        two frames using tf.
        """   
        
        try:
            self.listener.waitForTransform(target, origin, rospy.Time(0), rospy.Duration(4.0))
            trans, rot = self.listener.lookupTransform(target, origin, rospy.Time(0))
            return [trans, rot]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to recieve the transform for {} to {}".format(origin, target))
            return [None, None]
        
    def update(self, cur_state):
        """
        Update the function when called from the main script.
        """
        
        # Get the trimmed down joint positions
        q_left = [0.0]*self.num_joints
        q_right = [0.0]*self.num_joints
        for i in range(0,self.num_joints):
            q_left[i] = cur_state.left_joint_state.position[self.left_index[i]]
            q_right[i] = cur_state.right_joint_state.position[self.right_index[i]]
        
        # Get the individual arm Jacobians (TODO: improve this)
        J_left = solutions.solve_boxbot_6dof_left(q_left)
        J_right = solutions.solve_boxbot_6dof_right(q_right)
            
        # Determine relavant frame transformations
        if self.master == "left":
            J_A = J_left
            J_B = J_right
            trans_21 = self.lookup_frame_transform(self.left_eef, self.left_base)
            trans_24 = self.lookup_frame_transform(self.left_eef, self.right_base)
            trans_23 = self.lookup_frame_transform(self.left_eef, self.right_eef)
        elif self.master == "right":
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
        J_R = self.get_relative_jacobian(J_A, J_B, R_21, R_24, p_23)

        return [J_left, J_right, J_R], "ok"
