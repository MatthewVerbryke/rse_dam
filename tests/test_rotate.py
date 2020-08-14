#!/usr/bin/env python

import sys
from deliberative.adapter import *
from deliberative.trajectories import *
from habitual.test_moveit import *
from habitual.reachability import *

REF_FRAME = "origin_point"

def rotate_test():
    
    start_pose = Pose()
    start_pose.position.x = 0.24
    start_pose.position.y = 0.0
    start_pose.position.z = 0.2659
    start_pose.orientation.x = 0.707107
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 0.707107
    
    goal_pose = Pose()
    goal_pose.position.x = 0.24
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.2659
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = -0.707107
    goal_pose.orientation.z = 0.707107
    goal_pose.orientation.w = 0.0
    
    arm_pose = Pose()
    arm_pose.position.x = 0.24
    arm_pose.position.y = -0.128
    arm_pose.position.z = 0.2659
    arm_pose.orientation.x = -0.707107
    arm_pose.orientation.y = 0.0
    arm_pose.orientation.z = 0.0
    arm_pose.orientation.w = 0.707107
    
    speed = 0.05 #rad/s
    lift_height = 0.1
    
    trajectory, timestamps = create_in_place_rotation_trajectory(start_pose, goal_pose, speed, REF_FRAME)
    offset = compute_eef_offset(start_pose, arm_pose)
    poses = adapt_arm_poses(trajectory, offset)
    interface = MoveItHabitualModule()
    
    for pose in poses.poses:
        pose_req = PoseStamped()
        pose_req.header.seq = 1
        pose_req.header.stamp.secs = 1
        pose_req.header.stamp.nsecs = 1
        pose_req.header.frame_id = REF_FRAME
        pose_req.pose = pose
        
        reachable = is_pose_reachable(pose, "right")
        joint_state = interface.ik_solve(pose_req)
        print joint_state
    
        
rotate_test()
        
