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
        self.cycle_time = rospy.Duration(rate)
        
        # Controller variables
        self.i = -1
        self.start = None
        self.end = None
        self.new_trajectory = False
        self.executing = False
        self.command = JointState()
        self.trajectory = JointTrajectory()
        self.cur_goal = JointTrajectoryPoint()
        
    def input_new_trajectory(self, goal_trajectory):
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
            
        self.trajectory = goal_trajectory
        self.new_trajectory = True
        
        return True
        
    def reset(self):
        """
        Reset the controller to the starting state
        """
        
        self.i = -1
        self.executing = False
        self.start = None
        self.end = None
        self.command = JointState()
        self.trajectory = JointTrajectory()
        self.cur_goal = JointTrajectoryPoint()
        
    def update(self, feedback):
        """
        Run through one loop of the controller when executing or waiting
        to execute.
        """
        
        # Setup up for execution of new trajectory
        if self.new_trajectory:
            rospy.loginfo("Executing new trajectory")
            self.i = -1
            self.new_trajectory = False
            self.executing = True
            
            # Handle start state/time
            self.start = self.trajectory.header.stamp
            if self.start.secs == 0 and self.start.nsecs == 0:
                self.start = rospy.Time.now() + self.cycle_time #<-- this may be the cause of the "jump" at the beginning of trajectory execution.
            self.end = self.start
            self.command = feedback
            
        # Trajectory execution
        if self.executing:
            
            # Wait if it isn't yet time to start
            if rospy.Time.now() + self.cycle_time < self.start:
                return 0, None
            
            # If we are past the current end time set up the next trajectory point
            elif rospy.Time.now() + self.cycle_time >= self.end:
                self.i += 1
                
                # Handle last waypoint in the trajectory
                if self.i > self.num_traj_points:
                    self.reset()
                    return 2, None
                    
                self.cur_goal = self.trajectory[self.i].position
                self.end = self.trajectory[self.i].time_from_start
            
            # Control towards the current goal 
            elif rospy.Time.now() + self.cycle_time < self.end:
                for j in range(0,self.num_joints):
                    error = feedback[j] - cur_goal[j]
                    velocity = abs(error[j]/(self.rate*(self.end - rospy.Time.now().to_sec())))
                    if abs(error) > 0.001:
                        if error > velocity:
                            cmd = velocity
                        elif error < -velocity:
                            cmd = -velocity
                        self.command.position[i] += cmd
                    else:
                        pass
        
                return 1, self.command
