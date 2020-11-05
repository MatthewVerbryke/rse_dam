#!/usr/bin/env python

"""
  A controller object using the relative Jacobian method for coordinated
  dual-arm control, specifically using the compact relative Jacobian 
  expression presented in Jamisola et al.
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import math
import traceback

from geometry_msgs.msg import Pose
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from tf import transformations

from csa_msgs.msg import TimedPoseArray


class RelativeJacobianController(object):
    """
    A Relative Jacobian Controller python object.
    """
    
    def __init__(self, joint_names, rate):
        
        # Controller Parameters
        self.joint_names = joint_names
        self.num_joints = len(joint_names) #<-- for a single arm
        self.rate = rate
        self.cycle_time = rospy.Duration(1./rate)
        self.dt = 1./rate
        
        # Controller Variables
        self.i = 0
        self.start = None
        self.end = None
        self.executing = False
        self.trajectory = TimedPoseArray()
        self.desired = Pose()
        
        # Gains
        self.kp = [1.0, 1.0, 15.0, 1.0, 1.0, 1.0]
        self.kp2 = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
        
    def input_new_trajectory(self, trajectory):
        """
        Check and store newly recieved trajectory.
        """
        
        # Make sure there is actually waypoints in the trajectories
        if not trajectory.poses or not trajectory.time_from_start:
            rospy.logerr("Master arm trajectory waypoint list is empty")
            return False
        
        # Store Trajectories
        self.num_traj_points = len(trajectory.poses)/2
        self.trajectory_a = TimedPoseArray()
        self.trajectory_a.poses = trajectory.poses[0:self.num_traj_points]
        self.trajectory_a.time_from_start = trajectory.time_from_start[0:self.num_traj_points]
        self.trajectory_r = TimedPoseArray()
        self.trajectory_r.poses = trajectory.poses[self.num_traj_points:]
        self.trajectory_r.time_from_start = trajectory.time_from_start[self.num_traj_points:]
        
        # Handle start time
        self.start = self.trajectory.header.stamp
        if self.start.secs == 0 and self.start.nsecs == 0:
            self.start = rospy.Time.now() + rospy.Duration(3.0) 
        
        # Setup new trajectory
        self.i = 0
        self.desired_a = self.trajectory_a.poses[0]
        self.desired_r = self.trajectory_r.poses[0]
        self.end = self.start + self.trajectory_a.time_from_start[0]
        self.xa_dot = [0.0]*6
        self.xr_dot = [0.0]*6
        
        return True
        
    def remap_pose(self, pose, ref_pose):
        """
        Remap a pose into the frame of a reference pose. Assumes 'pose' 
        and 'ref_pose' are both represented w.r.t. the same 'global'
        reference frame.
        """
        
        # Get position and orientation of the two poses
        q_pose = [pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w]
        p_pose = np.array([[pose.position.x],
                           [pose.position.y],
                           [pose.position.z]])
        
        q_ref = [ref_pose.orientation.x,
                 ref_pose.orientation.y,
                 ref_pose.orientation.z,
                 ref_pose.orientation.w]
        p_ref = np.array([[ref_pose.position.x],
                          [ref_pose.position.y],
                          [ref_pose.position.z]])
        
        # Convert pose and reference pose into transformation matrices
        T_pose = transformations.quaternion_matrix(q_pose)
        T_pose[0:3,3:4] = p_pose
        
        T_ref = transformations.quaternion_matrix(q_ref)
        T_ref[0:3,3:4] = p_ref
        
        # Invert reference pose matrix
        T_ref_inv = transformations.inverse_matrix(T_ref)
        
        # Calculate transformation
        T_rel = np.matmul(T_ref_inv,T_pose)
        
        # Convert back into ROS Pose
        q_rel = transformations.quaternion_from_matrix(T_rel)
        rel_pose = Pose()
        rel_pose.position.x = T_rel[0,3]
        rel_pose.position.y = T_rel[1,3]
        rel_pose.position.z = T_rel[2,3]
        rel_pose.orientation.x = q_rel[0]
        rel_pose.orientation.y = q_rel[1]
        rel_pose.orientation.z = q_rel[2]
        rel_pose.orientation.w = q_rel[3]
        
        return rel_pose
    
    def get_delta_from_dT(self, T0, T1):
        """
        Determine the differential motion between two frames represented
        as homogeneous transformation matricies. The result also 
        represented as a transfomation matrix
        
        Adapted from 'tr2delta.m' function found in the repository:
        https://github.com/petercorke/robotics-toolbox-matlab
        """
        
        # Determine increment from T0 to T1 (TD)
        T0_inv = np.linalg.inv(T0)
        TD = np.matmul(T1, T0_inv)
        
        # Get necessary components for delta vector
        TD_pos = T1[0:3,3:4] - T0[0:3,3:4]
        TD_rot = TD[0:3,0:3]
        TD_neye = TD_rot - np.eye(3)
        skew_sym_vec = np.array([[0.5*(TD_neye[2,1] - TD_neye[1,2])],
                                 [0.5*(TD_neye[0,2] - TD_neye[2,0])],
                                 [0.5*(TD_neye[1,0] - TD_neye[0,1])]])
                                     
        # Determine delta vector
        delta = np.vstack((TD_pos, skew_sym_vec))
        
        return delta
    
    def pose_to_T_matrix(self, pose):
        """
        Convert a 'geometry_msgs/Pose' object into a 4x4 homogeneous 
        transformation matrix.
        """
        
        # Get elements out of pose
        position = [[pose.position.x],
                    [pose.position.y],
                    [pose.position.z]]
        orientation = [pose.orientation.x,
                       pose.orientation.y,
                       pose.orientation.z,
                       pose.orientation.w]
                       
        # Convert and package into 4x4 array
        T = transformations.quaternion_matrix(orientation)
        T[0:3,3:4] = position
        
        return T
        
    def update(self, state):
        """
        Use feedback of system kinematics to determine the joint commands
        for the current controller loop.
        """
        
        # Get the current time
        time_now = rospy.Time.now()
        
        # If before start time, do wait
        if time_now + self.cycle_time < self.start:
            status = 0
            cmd_a = None
            cmd_b = None
            
        else:
            
            # Determine commands to be sent if end time has not been reached
            if time_now + self.cycle_time <= self.end:
                cmd_a, cmd_b, status = self.calculate_command(state)
            
            # Handle the end point
            if time_now + self.cycle_time > self.end:
                self.i += 1
                if self.i <= self.num_traj_points-1:
                    self.set_next_waypoint()
                    cmd_a = None
                    cmd_b = None
                    status = 1
                    #if self.i >=12:
                        #status = 2
                else:
                    cmd_a = None
                    cmd_b = None
                    status = 2
            
        return cmd_a, cmd_b, status
                
    def set_next_waypoint(self):
        """
        Determine the next waypoint for the controller to which to move
        the arms.
        """
        
        self.desired_a = self.trajectory_a.poses[self.i]
        self.desired_r = self.trajectory_r.poses[self.i]
        next_t = self.trajectory_a.time_from_start[self.i]
        self.end = self.start + next_t
        
    def calculate_command(self, state):
        """
        Determine the command for the current control loop and robot 
        state.
        """
        
        # Extract relevant state information
        joint_pos_a = list(state[0].position)
        Ja = state[1]
        eef_a = state[2]
        joint_pos_b = list(state[3].position)
        Jb = state[4]
        eef_b = state[5]
        Jr = state[6]
        
        # print "eef a"
        # print eef_a
        # print ""
        # print "desired a"
        # print self.desired_a
        # print ""
        
        # Determine desired change in master's pose (in 'task' space)
        Ta = self.pose_to_T_matrix(eef_a)
        Ta_des = self.pose_to_T_matrix(self.desired_a)
        xa_dot = self.get_delta_from_dT(Ta, Ta_des)
        delta_a = np.array([[kp*a[0]/0.35] for kp,a in zip(self.kp,xa_dot)])
        # print "delta a"
        # print delta_a
        # print ""

        # Determine desired change in slave's pose (in master's eef frame)
        eef_b_ra = self.remap_pose(eef_b, eef_a)
        Tr = self.pose_to_T_matrix(eef_b_ra)
        Tr_des = self.pose_to_T_matrix(self.desired_r)
        xr_dot = self.get_delta_from_dT(Tr, Tr_des)
        delta_r = np.array([[kp*r[0]/0.35] for kp,r in zip(self.kp2,xr_dot)])
        # print "delta r"
        # print xr_dot
        # print ""
        
        # # Use relative and individual Jacobians to determine q_dot
        Jr_inv = np.linalg.pinv(Jr)
        Ns_proj = np.identity(2*self.num_joints) - np.matmul(Jr_inv, Jr)
        Ja_ext = np.hstack((Ja, np.zeros((6,self.num_joints))))
        Ja_ns = np.matmul(Ns_proj, np.linalg.pinv(Ja_ext))
        q_dot_full = np.matmul(Jr_inv, delta_r) + np.matmul(Ja_ns, delta_a)

        # Determine desired change in joint position for each arm
        q_dot_a = q_dot_full[0:self.num_joints].flatten().tolist()
        q_dot_b = q_dot_full[self.num_joints:].flatten().tolist()
        # print "q_dot_a"
        # print q_dot_a
        # print ""
        # print "q_dot_b"
        # print q_dot_b
        # print ""
        # print "arm a joint pose"
        # print joint_pos_a
        # print ""
        # print "arm b joint pose"
        # print joint_pos_b
        # print "--------------------------------------------------------"
        
        # Add changes to the commands for each arm
        status = 1
        command_a = [qa + dqa*self.dt for qa,dqa in zip(joint_pos_a, q_dot_a)]
        command_b = [qb + dqb*self.dt for qb,dqb in zip(joint_pos_b, q_dot_b)]
        #command_a[0] = 0.0
        #command_b[0] = 0.0
        
        # print "command a"
        # print command_a
        # print ""
        # print "command b"
        # print command_b
        # print ""
        # Store current error as previous error
        #self.xa_dot_prev = xa_dot
        #self.xr_dot_prev = xr_dot
        
        return command_a, command_b, status
