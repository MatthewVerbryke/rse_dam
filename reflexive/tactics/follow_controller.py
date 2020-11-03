#!/usr/bin/env python

"""
  A simple joint-space, position feedback controller with set-point
  interpolation.
  
  Modified from "follow_controller.py" in https://github.com/Interbotix/arbotix_ros
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test
"""


import traceback

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FollowController(object):
    """
    A Follow Controller python object.
    """
    
    def __init__(self, joint_names, rate):
        
        # Controller Parameters
        self.joint_names = joint_names
        self.num_joints = len(joint_names)
        self.rate = rate
        self.cycle_time = rospy.Duration(1./rate)
        self.dt = 1./rate
        
        # Controller variables
        self.i = -1
        self.start = None
        self.end = None
        self.executing = False
        self.command = JointState()
        self.command.position = [0.0]*self.num_joints
        self.trajectory = JointTrajectory()
        self.cur_goal = JointTrajectoryPoint()
        
        # Joint gain lists
        self.p_gains = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        
    def input_new_trajectory(self, trajectory):
        """
        Recieve new trajectory and check it.
        """
        
        # Make sure joint names match
        if set(trajectory.joint_names) != set(self.joint_names):
            rospy.logerr("Joint names in trajectory and internal controller list do not match")
            return False
        
        # Make sure there is a trajectory in the goal message
        if not trajectory.points:
            rospy.logerr("Waypoint list empty in the given trajectory")
            return False
        
        # Store trajectory
        self.trajectory = trajectory
        self.num_traj_points = len(trajectory.points)
        
        # Handle start time
        self.start = self.trajectory.header.stamp
        if self.start.secs == 0 and self.start.nsecs == 0:
            self.start = rospy.Time.now() + rospy.Duration(3.0)
        
        # Setup new trajectory
        self.i = 0
        self.cur_goal = self.trajectory.points[self.i]
        self.desired = self.cur_goal.positions
        self.end = self.start + self.cur_goal.time_from_start
        
        return True
        
    def update(self, state):
        """
        Run through one loop of the controller when executing or waiting
        to execute.
        """
        
        # Get the current time
        time_now = rospy.Time.now()
        
        # Wait until the start time has been reached
        if time_now + self.cycle_time < self.start:
            status = 0
            command = None
            
        else:
            
            # Determine commands to be sent if end time has not been reached
            if time_now + self.cycle_time <= self.end:
                command, status = self.calculate_command(state)
        
            # Handle end time of current goal
            elif time_now + self.cycle_time >= self.end:
                self.i += 1
                if self.i <= self.num_traj_points-1:
                    self.set_next_waypoint()
                    command = None
                    status = 1
                else:
                    command = None
                    status = 2
                    
        return command, status
                
    def set_next_waypoint(self):
        """
        Determine the next waypoint for the controller to which to move
        the arms.
        """
        
        self.desired = self.trajectory.points[self.i].positions
        next_t = self.trajectory.points[self.i].time_from_start
        self.end = self.start + next_t
        
    def calculate_command(self, state):
        """
        Determine the command for the current control loop and robot 
        state.
        """
        
        # Get out current joint states of the arm
        joint_pos = list(state.position)
        command = joint_pos
        
        # Determine change in joint position
        q_err = [(d-c) for d,c in zip(self.desired,joint_pos)]
        dq = [g*x*self.dt for g,x in zip(self.p_gains, q_err)]
        for i in range(self.num_joints):
            if q_err[i] > 0.001 or q_err[i] < -0.001:
                cmd = q_err[i]
                top = dq[i]
                if cmd > top:
                    cmd = top
                elif cmd < -top:
                    cmd = -top
                command[i] += cmd
                status = 1
        
        return command, status
