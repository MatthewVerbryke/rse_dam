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
    Follow controller python object
    """
    
    def __init__(self, joint_names, rate):
        
        # Controller parameters
        self.joint_names = joint_names
        self.num_joints = len(joint_names)
        self.rate = rate
        self.cycle_time = rospy.Duration(0.01)
        
        # Controller variables
        self.i = -1
        self.start = None
        self.end = None
        self.executing = False
        self.command = JointTrajectoryPoint()
        self.command.positions = [0.0]*self.num_joints
        self.trajectory = JointTrajectory()
        self.cur_goal = JointTrajectoryPoint()
        
    def input_new_trajectory(self, goal_trajectory, feedback):
        """
        Recieve new trajectory and check it.
        """
        
        # Make sure joint names match
        if set(goal_trajectory.joint_names) != set(self.joint_names):
            rospy.logerr("Joint names in trajectory and internal controller list do not match")
            return False
        
        # Make sure there is a trajectory in the goal message
        if not goal_trajectory.points:
            rospy.logerr("Waypoint list empty in the given trajectory")
            return False
        
        # Store trajectory
        self.trajectory = goal_trajectory
        self.num_traj_points = len(goal_trajectory.points)
        
        # Handle start time
        self.start = self.trajectory.header.stamp
        if self.start.secs == 0 and self.start.nsecs == 0:
            self.start = rospy.Time.now() + self.cycle_time #<-- this may be the cause of the "jump" at the beginning of trajectory execution.
        
        # Setup new trajectory
        self.i = 0
        self.cur_goal = self.trajectory.points[self.i]
        self.desired = self.cur_goal.positions
        self.end = self.start + rospy.Duration(self.cur_goal.time_from_start)
        self.last = feedback
        
        # Set execution flag
        rospy.loginfo("Executing new trajectory")
        self.executing = True
        
        return True
        
    def reset(self):
        """
        Reset the controller to the starting state
        """
        
        self.i = -1
        self.executing = False
        self.start = None
        self.end = None
        self.command = JointTrajectoryPoint()
        self.command.positions = [0.0]*self.num_joints
        self.trajectory = JointTrajectory()
        self.cur_goal = JointTrajectoryPoint()
        
    def update(self, last):
        """
        Run through one loop of the controller when executing or waiting
        to execute.
        """
        
        
        
        # Wait until the start time has been reached
        if rospy.Time.now() + self.cycle_time < self.start:
            return 0, None
        
        # Handle end of waypoints
        if rospy.Time.now() + self.cycle_time >= self.end:
            self.i += 1
            
            # Signal completion of trajectory execution
            if self.i > self.num_traj_points:
                rospy.loginfo("Trajectory execution completed")
                self.reset()
                return 2, None
                
            # Update current goal (goes into next conditional statement)
            else:
                self.cur_goal = self.trajectory.points[self.i]
                self.desired = self.cur_goal.positions
                self.end = self.start + rospy.Duration(self.cur_goal.time_from_start)
        
        # Determine command to send to joints
        if rospy.Time.now() + self.cycle_time < self.end:
            error = [(d-c) for d,c in zip(self.desired,last)]
            velocity = [abs(x / (self.rate*(self.end-rospy.Time.now()).to_sec())) for x in error]
            for i in range(self.num_joints):
                if error[i] > 0.001 or error[i] < -0.001:
                    cmd = error[i] 
                    top = velocity[i]
                    if cmd > top:
                        cmd = top
                    elif cmd < -top:
                        cmd = -top
                    last[i] += cmd
                else:
                    velocity[i] = 0
            return 1, last
