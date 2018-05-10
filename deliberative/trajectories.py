#!/usr/bin/env python

"""
  Functions to create template based MoveIt! trajectories.
  
  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.
  
  TODO: improve documentation across the board
"""

import copy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import math
import rospy


def create_simple_move_trajectory(start_pose, goal_pose, speed, ref_frame):
    """
    Create a simple trajectory in which the target object moves from the
    start pose to goal pose at the given speed, with no change in the 
    object's orientation. Assumes no obstacles are on the path for 
    simplicity.
    """
    
    timestep = 0.1
    
    # Get distances between the start and end points
    delta_x = goal_pose.position.x - start_pose.position.x
    delta_y = goal_pose.position.y - start_pose.position.y
    delta_z = goal_pose.position.z - start_pose.position.z
    traverse_len = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
    
    # Calculate the number of waypoints along the path (round to next-highest int)
    traverse_time = traverse_len/speed
    traverse_wp_len = int(math.ceil(traverse_time/timestep))
    
    # Fill out array header
    trajectory = PoseArray()
    trajectory.header.seq = 0
    trajectory.header.frame_id = 1
    
    # Setup loop   
    x_step = (goal_pose.position.x - start_pose.position.x)/traverse_wp_len
    y_step = (goal_pose.position.y - start_pose.position.y)/traverse_wp_len
    z_step = (goal_pose.position.z - start_pose.position.z)/traverse_wp_len
    pose_list = [None]*(traverse_wp_len+1)
    timestamps = []
    
    # Fill out poses and timesteps
    for i in range(0, traverse_wp_len):
        
        # Get pose
        pose_i = Pose()
        pose_i.position.x = start_pose.position.x + x_step*i
        pose_i.position.y = start_pose.position.y + y_step*i
        pose_i.position.z = start_pose.position.z + z_step*i
        pose_i.orientation = start_pose.orientation
        pose_list[i] = pose_i
        
        # Get timestamp
        sec, nsec = get_stamp(i*timestep)
        timestamps.append({"secs": sec, "nsecs": nsec})        
             
    # Place the goal point at the end as well
    temp_pose = copy.deepcopy(goal_pose)
    pose_list[traverse_wp_len] = temp_pose
    sec, nsec = get_stamp((i+1)*timestep)
    timestamps.append({"secs": sec, "nsecs": nsec})
    
    # Package result
    trajectory.poses = pose_list
    
    return trajectory, timestamps
    
def create_pick_and_place_trajectory(start_pose, goal_pose, lift_height, speed, ref_frame):
    """
    Creates a simple pick and place trajectory composed of three smaller
    sub-trajectories, with no change in the orientation of the target
    object. It involves picking up the target object by a set height off
    the surface, moving to a point at the same set height above the goal 
    position, and then placing the object at the goal position. Assumes
    no obstacles are on the path for simplicity.
    
    TODO: REWORK TO USE THE SIMPLER TRAJECTORY FOUND ABOVE AND WITHOUT 
    POSESTAMPED.
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
            pose_j = PoseStamped()
            pose_j.header.seq = j
            pose_j.header.stamp = j*timestep
            pose_j.header.frame_id = ref_frame
            pose_j.pose.position.x = points[i].position.x + x_step*j
            pose_j.pose.position.y = points[i].position.y + y_step*j
            pose_j.pose.position.z = points[i].position.z + z_step*j
            pose_j.pose.orientation = start_pose.orientation
            pose_list[j] = pose_j

        # Place the goal point at the end as well
        temp_pose = PoseStamped()
        temp_pose.header.seq = j+1
        temp_pose.header.stamp = rospy.Time.from_seconds((j+1)*timestep)
        temp_pose.header.frame_id = ref_frame
        temp_pose.pose = copy.deepcopy(points[i+1])
        pose_list[list_lens[i]] = temp_pose
        
        # Package result
        trajectory.poses = pose_list
        trajectories[i] = trajectory
    
    return trajectories
    
def get_stamp(float_time):
    """
    Convert a floating-point representation of time in seconds into a 
    ros_msgs 'stamp' style representation.
    """

    # Convert to time in seconds (float) to time in nanoseconds (int)
    nsec = rospy.Time.from_sec(float_time)

    # Get the number of 'whole' seconds
    sec = int(float_time)
    
    return sec, nsec.nsecs
