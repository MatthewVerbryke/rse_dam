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


import numpy as np

from geometry_msgs.msg import Pose
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
        self.cycle_time = rospy.Duration(rate)
        
        # Controller Variables
        self.master = None
        self.waypoint = 0
        self.num_waypoints = 0
        self.start = 0.0
        self.end = 0.0
        self.trajectory = TimedPoseArray()
        self.goal_pose = Pose()
        
    def check_new_trajectory(self, trajectory, master):
        """
        Check and store newly recieved trajectory.
        """
        
        # Make sure there is actually a trajectory int the input
        if not trajectory.Pose or not trajectory.time_from_start:
            rospy.logerr("Waypoint list empty in the given trajectory")
            return False
        
        # Determine if this arm is "master" or "slave" for this trajectory
        self.master = master
        
        # Prepare trajectory for execution
        self.trajectory = trajectory
        self.num_waypoints = len(trajectory.poses)
        self.goal_pose = trajectory.poses[0]
        self.end = trajectory.time_from_start[0]
        
        # Handle execution start time
        if trajectory.header.secs =ww= 0 and trajectory.header.nsecs == 0:
            self.start = rospy.Time.now()
        else
            self.start = trajectory.header
        
        return True
        
    def calculate_desired_vel(self, cur_pose):
        """
        Using the current pose of the end-effector, calculate the new 
        goal velocity for the current controller loop.
        """
        
        # Convert quaternions to euler angles
        cur_angles = transformations.euler_from_quaternion([cur_pose.orientation.x,
                                                            cur_pose.orientation.y,
                                                            cur_pose.orientation.z,
                                                            cur_pose.orientation.w])
        goal_angles = transformations.euler_from_quaternion([self.goal_pose.orientation.x,
                                                             self.goal_pose.orientation.y,
                                                             self.goal_pose.orientation.z,
                                                             self.goal_pose.orientation.w])
        
        # Calculate error between current pose and goal pose
        error = [goal_pose.position.x - cur_pose.position.x,
                 goal_pose.position.y - cur_pose.position.y,
                 goal_pose.position.z - cur_pose.position.z,
                 goal_angles[0] - cur_angles[0],
                 goal_angles[1] - cur_angles[1],
                 goal_angles[2] - cur_angles[2]]
        
        # Get velocities for current step
        vel = error/(self.rate*(self.end - rospy.Time.now().to_secs()))
        
        return vel
        
    def update(self, Ja, Jr, cur_pose, cur_joint_angles):
        """
        Use feedback of system kinematics to determine the joint commands
        for the current controller loop.
        """
        
        # If the start time has not yet arrived, wait
        if rospy.Time.now() + self.cycle_time < self.start:
            return "waiting", None
            
        # If current time is pass the current ending time...
        if rospy.Time.now() + self.cycle_time > self.end:
            
            # Signal success if this is the last waypoint
            if self.waypoint == self.num_waypoints:
                return "success", None
            
            # Otherwise cycle to the next waypoint
            else:
                self.waypoint += 1
                self.goal_pose = self.trajectory.poses[self.waypoint]
                self.end = self.trajectory.time_from_start[self.waypoint]
    
        # If before the ending time, calculate next command
        if rospy.Time.now() + self.cycle_time <= self.end:
            
            # Determine desired joint velocities
            if master:
                x_dot = self.calculate_desired_vel(cur_pose)
                q_dot = np.matmul(Ja, x_dot)
            else:
                x_dot = self.calculate_desired_vel(cur_pose)
                Ja_inv = np.linalg.pinv(Ja)
                Na = np.eye[6] - np.matmul(Ja_inv, Ja)
                Jr_na = np.matmul(Jr, Na)
                Jr_na_inv = np.linalg.pinv(Jr_na)
                q_dot_d = np.matmul(Jr_na_inv, x_dot)
                q_dot = q_dot_d[(self.num_joints+1):]
                
            # Determine desired joint position command
            command = cur_joint_angles + q_dot
            
            return "executing", command
            
    def reset_controller(self):
        """
        Reset controller variables to prepare for a new trajectory to 
        execute.
        """
        
        # Reset variables
        self.master = None
        self.waypoint = 0
        self.num_waypoints = 0
        self.start = 0.0
        self.end = 0.0
        self.trajectory = TimedPoseArray()
        self.goal_pose = Pose()
        
        return "reset", None
