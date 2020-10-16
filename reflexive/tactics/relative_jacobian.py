#!/usr/bin/env python

"""
  A controller object using the relative Jacobian method for coordinated
  dual-arm control, specifically using the compact relative Jacobian 
  expression presented in Jamisola et al. and Yan et al.
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test everything
"""


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
        self.num_joints = len(joint_names)
        self.rate = rate
        self.cycle_time = rospy.Duration(0.01)
        
        # Controller Variables
        self.i = 0
        self.start = None
        self.end = None
        self.executing = False
        self.trajectory = TimedPoseArray()
        self.desired = Pose()
        
    def input_new_trajectory(self, trajectory, feedback, master):
        """
        Check and store newly recieved trajectory.
        """
        
        # Make sure there is actually a trajectory int the input
        if not trajectory.poses or not trajectory.time_from_start:
            rospy.logerr("Waypoint list empty in the given trajectory")
            return False
        
        # Determine if this arm is "master" or "slave" for this trajectory
        self.master = master
        
        # Store Trajectory
        self.trajectory = trajectory
        self.num_traj_points = len(trajectory.poses)
        self.goal_pose = trajectory.poses[0]
        self.end = trajectory.time_from_start[0]
        
        # Handle start time
        self.start = self.trajectory.header.stamp
        if self.start.secs == 0 and self.start.nsecs == 0:
            self.start = rospy.Time.now() + self.cycle_time #<-- this may be the cause of the "jump"
        
        # Setup new trajectory
        self.i = 0
        self.desired = trajectory.poses[0]
        self.end = self.start + rospy.Duration(trajectory.time_from_start[0])
        self.last = feedback
        
        return True
        
    def remap_pose(self, pose, ref_pose):
        """
        Remap a pose into the frame of a reference pose. Assumes pose
        ref_pose are both represented w.r.t. the same 'global' reference
        frame.
        """
        
        # Get position and orientation of the two poses
        q_pose = [pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w]
        p_pose = np.array([pose.position.x]
                          [pose.position.y]
                          [pose.position.z])
        
        q_ref = [ref_pose.orientation.x,
                 ref_pose.orientation.y,
                 ref_pose.orientation.z,
                 ref_pose.orientation.w]
        p_ref = np.array([ref_pose.position.x]
                         [ref_pose.position.y]
                         [ref_pose.position.z])
        
        # Convert pose and reference pose into transformation matrices
        R_pose = transfromations.quaternion_matrix(q_pose)
        T_pose = np.identity(4)
        T_pose[0:3,0:3] = R_pose
        T_pose[0:3,3:4] = p_pose
        
        R_ref = transfromations.quaternion_matrix(q_ref)
        T_ref = eye(4)
        T_ref[0:3,0:3] = R_ref
        T_ref[0:3,3:4] = p_ref
        
        # Invert reference pose matrix
        T_ref_inv = transformations.inverse_matrix(T_ref)
        
        # Calculate transformation
        T_rel = T_ref_inv*T_pose
        
        # Convert back into pose
        R_rel = T_rel[0:3,0:3]
        q_rel = transformations.quaternion_from_matrix(R_rel)
        rel_pose = Pose()
        rel_pose.position.x = T_rel[0,3]
        rel_pose.position.y = T_rel[1,3]
        rel_pose.position.z = T_rel[2,3]
        rel_pose.orientation.x = q_rel[0]
        rel_pose.orientation.y = q_rel[1]
        rel_pose.orientation.z = q_rel[2]
        rel_pose.orientation.w = q_rel[3]
        
        return rel_pose
        
    def calculate_desired_vel(self, cur_pose, goal_pose):
        """
        Using the current pose of the end-effector, calculate the new 
        goal velocity for the current controller loop.
        """
        
        # Convert quaternions to euler angles
        cur_angles = transformations.euler_from_quaternion([cur_pose.orientation.x,
                                                            cur_pose.orientation.y,
                                                            cur_pose.orientation.z,
                                                            cur_pose.orientation.w])
        goal_angles = transformations.euler_from_quaternion([goal_pose.orientation.x,
                                                             goal_pose.orientation.y,
                                                             goal_pose.orientation.z,
                                                             goal_pose.orientation.w])
        
        # Calculate error between current pose and goal pose
        error = [goal_pose.position.x - cur_pose.position.x,
                 goal_pose.position.y - cur_pose.position.y,
                 goal_pose.position.z - cur_pose.position.z,
                 goal_angles[0] - cur_angles[0],
                 goal_angles[1] - cur_angles[1],
                 goal_angles[2] - cur_angles[2]]
        
        # Get velocities for current step
        dt = (self.rate*(self.end.to_sec() - rospy.Time.now().to_sec()))
        vel = [x/dt for x in error]
        
        return vel
        
    def update(self, state):
        """
        Use feedback of system kinematics to determine the joint commands
        for the current controller loop.
        """
        
        # Extract relevant state information
        if self.master:

            cur_joint_angles = state[0]
            Ja = state[1]
            cur_pose = state[3]
        else:
            cur_joint_angles = state[0]
            Ja = state[2]
            cur_pose = self.remap_pose(state[3], state[4])
            Jr = state[5]
        
        # If the start time has not yet arrived, wait
        if rospy.Time.now() + self.cycle_time < self.start:
            status = 0
            command = None
            
        # If current time is pass the current ending time...
        if rospy.Time.now() + self.cycle_time > self.end:
            self.i += 1
            
            # Signal success if this is the last waypoint
            if self.i == self.num_traj_points:
                status = 2
                command = None
            
            # Cycle to the next waypoint
            else:
                self.desired = self.trajectory.poses[self.i]
                next_t = self.trajectory.time_from_start[self.i]
                self.end = self.start + rospy.Duration(next_t)
    
        # Determine command to send to joints
        if rospy.Time.now() + self.cycle_time <= self.end:
            
            # Determine desired joint velocities
            if self.master:
                x_dot = self.calculate_desired_vel(cur_pose, self.desired)
                Ja_inv = np.linalg.pinv(Ja) # TODO: add warning somewhere if we hit a singularity?
                q_dot = np.matmul(Ja_inv, x_dot)
            else:
                x_dot = self.calculate_desired_vel(cur_pose, self.desired)
                Ja_inv = np.linalg.pinv(Ja) # TODO: add warning somewhere if we hit a singularity?
                Na = np.eye[6] - np.matmul(Ja_inv, Ja)
                Jr_na = np.matmul(Jr, Na)
                Jr_na_inv = np.linalg.pinv(Jr_na)
                q_dot_d = np.matmul(Jr_na_inv, x_dot)
                q_dot = q_dot_d[(self.num_joints+1):]
                
            # Determine desired joint position command
            joint_poses = [sum(x) for x in zip(cur_joint_angles, q_dot)]
            
            # Package into message
            status = 1
            command = JointState()
            command.name = self.joint_names
            command.position = joint_poses
            
            return 1, command
            
    def reset(self):
        """
        Reset controller variables to prepare for a new trajectory to 
        execute.
        """
        
        # Reset variables
        self.master = None
        self.i = 0
        self.executing = False
        self.num_traj_points = 0
        self.start = None
        self.end = None
        self.trajectory = TimedPoseArray()
        self.desired = Pose()
