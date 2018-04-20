#!/usr/bin/env python

"""
  TODO: module description
  
  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.
  
  TODO: improve documentation across the board
"""

import copy

from geometry_msgs.msg import Pose, PoseArray
import math
import rospy


def create_simple_move_trajectory(start_pose, goal_pose, lift_height, speed):
    """
    Creates a simple pick and place trajectory composed of three smaller
    sub-trajectories, with no change in the orientation of the target
    object. It involves picking up the target object by a set height off
    the surface, moving to a point at the same set height above the goal 
    position, and then placing the object at the goal position. Assumes
    no obstacles are on the path for simplicity.
    """
    
    timestep = 0.1
    trajectories = [None]*3
    
    # Create intermediate waypoints
    waypoint_1 = copy.deepcopy(start_pose)
    waypoint_2 = copy.deepcopy(goal_pose)
    waypoint_1.position.z += lift_height
    waypoint_2.position.z += lift_height

    # Get distances between the waypoints 1 and 2 ('traverse')
    delta_x = waypoint_2.position.x - waypoint_1.position.x
    delta_y = waypoint_2.position.y - waypoint_1.position.y
    delta_z = waypoint_2.position.z - waypoint_1.position.z
    traverse_len = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
    
    # Calculate the number of waypoints along each path (round to next-highest int)
    lift_time = lift_height/speed
    lift_wp_len = int(math.ceil(lift_time/timestep))
    traverse_time = traverse_len/speed
    traverse_wp_len = int(math.ceil(traverse_time/timestep))

    # Setup loop
    list_lens = [lift_wp_len, traverse_wp_len, lift_wp_len]
    points = [start_pose, waypoint_1, waypoint_2, goal_pose]

    # Create the trajectories for each subtrajectory
    for i in range(0,3):
        
        # Fill out array header
        trajectory = PoseArray()
        trajectory.header.seq = i
        trajectory.header.frame_id = 1

        # Setup nested loop
        x_step = (points[i+1].position.x - points[i].position.x)/list_lens[i]
        y_step = (points[i+1].position.y - points[i].position.y)/list_lens[i]
        z_step = (points[i+1].position.z - points[i].position.z)/list_lens[i]
        pose_list = [None]*(list_lens[i]+1)
        
        # Fill out poses
        for j in range(0, list_lens[i]):
            pose_j = Pose()
            pose_j.position.x = points[i].position.x + x_step*j
            pose_j.position.y = points[i].position.y + y_step*j
            pose_j.position.z = points[i].position.z + z_step*j
            pose_j.orientation = start_pose.orientation
            pose_list[j] = pose_j
            
        # Place the goal point at the end as well
        pose_list[list_lens[i]] = copy.deepcopy(points[i+1])
        
        # Package result
        trajectory.poses = pose_list
        trajectories[i] = trajectory
        
    return trajectories

