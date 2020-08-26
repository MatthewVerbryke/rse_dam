#!/usr/bin/env python

"""
  Jacobian updater for a robotic arm

  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.

  TODO: pure sympy implimentation might be too slow. Investigating other 
  approaches. Also need to check accuracy of current code's answer
  
"""


import numpy as np
import sympy as sp
from sympy import sin, cos, pi
from sympy.matrices import Matrix, eye, zeros


class JacobianSolver(object):
    """
    An object to persistently solve for the Jacobian matrix of a robotic
    arm during runtime
    
    NOTE: DH parameterization based on method found in John J. Craig's 
          "Introduction to Robotics, Mechanics and Control" 3rd edition
    """
    
    def __init__(self, DH_param):
        """
        Setup the required functions to get the Jacobian during runtime
        based on the DH parameters of the arm.
        """
        
        # Class variables
        self.num_joints = len(DH_param)
        self.J_func = zeros(6,self.num_joints)
        self.offsets = [None]*self.num_joints
        self.var_list = sp.symbols('q0:%d'%self.num_joints)
        
        # Storage
        T_i = [None]*self.num_joints
        
        # Get DH parameters for each link
        for i in range(0,self.num_joints):
            alpha = DH_param[i][0]
            a = DH_param[i][1]
            d = DH_param[i][2]
            theta = self.var_list[i];
            self.offsets[i] = DH_param[i][3]
            
            # Determine individual link transformation matrices
            T = Matrix([[cos(theta), -sin(theta), 0.0, a],
                        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                        [0.0, 0.0, 0.0, 1.0]])
            
            # Multiply this transformation to get the transformation to the ith joint
            if i == 0:
                T_i[0] = eye(4)*T
            else:
                T_i[i] = T_i[i-1]*T
            
        # Get the end effector transformation
        T_eef = T_i[self.num_joints-1]
            
        # Get out position functions from final transformation matrix
        p_eef = [T_eef[0,3], T_eef[1,3], T_eef[2,3]]
        
        # Get Jacobian component equations
        for i in range(0,self.num_joints):
            
            # Determine derivatives for linear Jacobian component
            for j in range(0,3):
                self.J_func[i, j] = sp.diff(p_eef[j], self.var_list[i])
                
            # Determine the rotational Jacobian components
            w_0i = T_i[i].col(2)
            for k in range(0,3):
                self.J_func[i, k+2] = w_0i[k]
                
    def get_current_jacobian(self, q):
        """
        Solve for the current Jacobian matrix given the current joint 
        values.
        """
        
        sub_list = [None]*self.num_joints
        
        # Setup substitution list with joint offsets
        for i in range(0,self.num_joints):
            q_offset = q[i] + self.offsets[i]
            sub_list[i] = (self.var_list[i], q_offset)
        
        # Solve for Jacobian
        J_sp = self.J_func.subs(sub_list)
            
        # Convert to numpy array
        J = np.array(J_sp).astype(float)
        
        return J
        
