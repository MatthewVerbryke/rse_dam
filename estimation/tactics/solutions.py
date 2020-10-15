#!/usr/bin/env python

"""
  Autogenerated function to determine the Jacobian matrix for a serial arm.
"""


from math import cos, sin, atan2, pi

import numpy as np


def solve_boxbot_6dof(q):
    
    q0 = q[0]
    q1 = q[1] - (pi/2 - atan2(48.25,141.8))
    q2 = q[2] - atan2(48.25,141.8)
    q3 = q[3]
    q4 = q[4]
    q5 = q[5]
    
    J = np.zeros((6, 6))
    J[0,0] = (-0.071*(sin(q0)*sin(q1)*sin(q2) - sin(q0)*cos(q1)*cos(q2))*cos(q3) - 0.071*sin(q3)*cos(q0))*sin(q4) + (-0.071*sin(q0)*sin(q1)*cos(q2) - 0.071*sin(q0)*sin(q2)*cos(q1))*cos(q4) - 0.1408*sin(q0)*sin(q1)*cos(q2) - 0.1408*sin(q0)*sin(q2)*cos(q1) - 0.15*sin(q0)*cos(q1)
    J[1,0] = (-0.071*sin(q1)*sin(q2)*cos(q0) + 0.071*cos(q0)*cos(q1)*cos(q2))*cos(q4) - 0.071*(-sin(q1)*cos(q0)*cos(q2) - sin(q2)*cos(q0)*cos(q1))*sin(q4)*cos(q3) - 0.1408*sin(q1)*sin(q2)*cos(q0) - 0.15*sin(q1)*cos(q0) + 0.1408*cos(q0)*cos(q1)*cos(q2)
    J[2,0] = (-0.071*sin(q1)*sin(q2)*cos(q0) + 0.071*cos(q0)*cos(q1)*cos(q2))*cos(q4) - 0.071*(-sin(q1)*cos(q0)*cos(q2) - sin(q2)*cos(q0)*cos(q1))*sin(q4)*cos(q3) - 0.1408*sin(q1)*sin(q2)*cos(q0) + 0.1408*cos(q0)*cos(q1)*cos(q2)
    J[3,0] = (0.071*(-sin(q1)*sin(q2)*cos(q0) + cos(q0)*cos(q1)*cos(q2))*sin(q3) - 0.071*sin(q0)*cos(q3))*sin(q4)
    J[4,0] = (-0.071*(-sin(q1)*sin(q2)*cos(q0) + cos(q0)*cos(q1)*cos(q2))*cos(q3) - 0.071*sin(q0)*sin(q3))*cos(q4) - (0.071*sin(q1)*cos(q0)*cos(q2) + 0.071*sin(q2)*cos(q0)*cos(q1))*sin(q4)
    J[5,0] = 0
    J[0,1] = (-0.071*(-sin(q1)*sin(q2)*cos(q0) + cos(q0)*cos(q1)*cos(q2))*cos(q3) - 0.071*sin(q0)*sin(q3))*sin(q4) + (0.071*sin(q1)*cos(q0)*cos(q2) + 0.071*sin(q2)*cos(q0)*cos(q1))*cos(q4) + 0.1408*sin(q1)*cos(q0)*cos(q2) + 0.1408*sin(q2)*cos(q0)*cos(q1) + 0.15*cos(q0)*cos(q1)
    J[1,1] = (-0.071*sin(q0)*sin(q1)*sin(q2) + 0.071*sin(q0)*cos(q1)*cos(q2))*cos(q4) - 0.071*(-sin(q0)*sin(q1)*cos(q2) - sin(q0)*sin(q2)*cos(q1))*sin(q4)*cos(q3) - 0.1408*sin(q0)*sin(q1)*sin(q2) - 0.15*sin(q0)*sin(q1) + 0.1408*sin(q0)*cos(q1)*cos(q2)
    J[2,1] = (-0.071*sin(q0)*sin(q1)*sin(q2) + 0.071*sin(q0)*cos(q1)*cos(q2))*cos(q4) - 0.071*(-sin(q0)*sin(q1)*cos(q2) - sin(q0)*sin(q2)*cos(q1))*sin(q4)*cos(q3) - 0.1408*sin(q0)*sin(q1)*sin(q2) + 0.1408*sin(q0)*cos(q1)*cos(q2)
    J[3,1] = (0.071*(-sin(q0)*sin(q1)*sin(q2) + sin(q0)*cos(q1)*cos(q2))*sin(q3) + 0.071*cos(q0)*cos(q3))*sin(q4)
    J[4,1] = (-0.071*(-sin(q0)*sin(q1)*sin(q2) + sin(q0)*cos(q1)*cos(q2))*cos(q3) + 0.071*sin(q3)*cos(q0))*cos(q4) - (0.071*sin(q0)*sin(q1)*cos(q2) + 0.071*sin(q0)*sin(q2)*cos(q1))*sin(q4)
    J[5,1] = 0
    J[0,2] = 0
    J[1,2] = (0.071*sin(q1)*sin(q2) - 0.071*cos(q1)*cos(q2))*sin(q4)*cos(q3) + (0.071*sin(q1)*cos(q2) + 0.071*sin(q2)*cos(q1))*cos(q4) + 0.1408*sin(q1)*cos(q2) + 0.1408*sin(q2)*cos(q1) + 0.15*cos(q1)
    J[2,2] = (0.071*sin(q1)*sin(q2) - 0.071*cos(q1)*cos(q2))*sin(q4)*cos(q3) + (0.071*sin(q1)*cos(q2) + 0.071*sin(q2)*cos(q1))*cos(q4) + 0.1408*sin(q1)*cos(q2) + 0.1408*sin(q2)*cos(q1)
    J[3,2] = -(-0.071*sin(q1)*cos(q2) - 0.071*sin(q2)*cos(q1))*sin(q3)*sin(q4)
    J[4,2] = -(0.071*sin(q1)*sin(q2) - 0.071*cos(q1)*cos(q2))*sin(q4) + (-0.071*sin(q1)*cos(q2) - 0.071*sin(q2)*cos(q1))*cos(q3)*cos(q4)
    J[5,2] = 0
    J[0,3] = 0
    J[1,3] = sin(q0)
    J[2,3] = sin(q0)
    J[3,3] = sin(q1)*cos(q0)*cos(q2) + sin(q2)*cos(q0)*cos(q1)
    J[4,3] = (-sin(q1)*sin(q2)*cos(q0) + cos(q0)*cos(q1)*cos(q2))*sin(q3) - sin(q0)*cos(q3)
    J[5,3] = -((-sin(q1)*sin(q2)*cos(q0) + cos(q0)*cos(q1)*cos(q2))*cos(q3) + sin(q0)*sin(q3))*sin(q4) + (sin(q1)*cos(q0)*cos(q2) + sin(q2)*cos(q0)*cos(q1))*cos(q4)
    J[0,4] = 0
    J[1,4] = -cos(q0)
    J[2,4] = -cos(q0)
    J[3,4] = sin(q0)*sin(q1)*cos(q2) + sin(q0)*sin(q2)*cos(q1)
    J[4,4] = (-sin(q0)*sin(q1)*sin(q2) + sin(q0)*cos(q1)*cos(q2))*sin(q3) + cos(q0)*cos(q3)
    J[5,4] = -((-sin(q0)*sin(q1)*sin(q2) + sin(q0)*cos(q1)*cos(q2))*cos(q3) - sin(q3)*cos(q0))*sin(q4) + (sin(q0)*sin(q1)*cos(q2) + sin(q0)*sin(q2)*cos(q1))*cos(q4)
    J[0,5] = 1
    J[1,5] = 0
    J[2,5] = 0
    J[3,5] = sin(q1)*sin(q2) - cos(q1)*cos(q2)
    J[4,5] = (sin(q1)*cos(q2) + sin(q2)*cos(q1))*sin(q3)
    J[5,5] = (sin(q1)*sin(q2) - cos(q1)*cos(q2))*cos(q4) - (sin(q1)*cos(q2) + sin(q2)*cos(q1))*sin(q4)*cos(q3)
    
    J_t = J.transpose() # <-- TODO: fix this silly error
    
    return J_t
    

if __name__ == "__main__":
    
    float_formatter = "{:.2f}".format
    np.set_printoptions(formatter={'float_kind':float_formatter})
    
    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    J = solve_jacobian(q)
    
    print J