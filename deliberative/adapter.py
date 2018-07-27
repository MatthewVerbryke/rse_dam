#!/usr/bin/env python

"""
  Trajectory adaptation funcitons for dual-arm manipulation.

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation
"""

import copy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import numpy as np
import tf


def compute_eef_offset(frame_1, frame_2):
    """
    Compute the offset (transfromation matrix) from frame_1 to frame_2.
    
    Adapted from:
    https://answers.ros.org/question/229329/what-is-the-right-way-to-inverse-a-transform-in-python/
    """
        
    frames = [frame_1, frame_2]
    tf_frames = []
        
    # Get the transformation matrix of each frame from the fixed reference frame
    for frame in frames:
        
        # If in a list found using LookupTransfrorm, use tf.transfrom
        if (type(frame)==list):
            trans_matrix = tf.transformations.translation_matrix(trans)
            rot_matrix = tf.transformations.quaternion_matrix(rot)
            tf_frames.append(tf.transformations.concatenate_matrices(trans_matrix, rot_matrix))
            
        # Use transfromations functions to get the frame info if frame is a pose
        elif (type(frame)==Pose) or (type(frame)==PoseStamped):
            tf_frames.append(pose_to_frame_matrix(frame))
            
        else:
            rospy.logerr("Frame {} is not an allowed type".format(len(tf_frames)+1)) #change
            return "error"
            
    # Invert the  first frame matrix
    matrix_1_inverse = tf.transformations.inverse_matrix(tf_frames[0])
        
    # Get the static transformation matrix from lead to follow frame
    offset = np.dot(matrix_1_inverse, tf_frames[1])

    return offset
        
def adapt_arm_poses(object_poses, offset):
    """
    Create an array of poses for the arm based on the object pose array,
    using the predetermined offset.
    """
        
    # Setup resultant PoseArray
    poses = PoseArray()
    poses.header = object_poses.header
        
    # Setup loop
    traj_wp_num = len(object_poses.poses)
    pose_list = [None]*traj_wp_num
    
    # Determine eef pose for each object pose
    for i in range(0,traj_wp_num):
        
        # Get object transform matrix at the time step
        object_matrix = pose_to_frame_matrix(object_poses.poses[i])
        
        # Determine the eef pose at the time step
        eef_matrix = np.dot(object_matrix, offset)
        pose_i = frame_matrix_to_pose(eef_matrix)
        
        # Store results
        pose_list[i] = pose_i
        
        # Store final arrays in PoseArray's
        poses.poses = pose_list
    
    return poses
        
def pose_to_frame_matrix(pose_in):
    """
    Convert a pose into a transformation matrix
    """
    
    # Get translation and quaternion rotation out of the pose
    if (type(pose_in)==PoseStamped):
        pose_trans = pose_in.pose.position
        pose_rot = pose_in.pose.orientation
    elif (type(pose_in)==Pose):
        pose_trans = pose_in.position
        pose_rot = pose_in.orientation
        
    # Parse them into numpy arrays
    trans = np.array([pose_trans.x, pose_trans.y, pose_trans.z])
    rot = np.array([pose_rot.x, pose_rot.y, pose_rot.z, pose_rot.w])
    
    # Convert to 'frame' (transformation matrix from origin)
    trans_matrix = tf.transformations.translation_matrix(trans)
    rot_matrix = tf.transformations.quaternion_matrix(rot)
    transformation_matrix = tf.transformations.concatenate_matrices(trans_matrix, rot_matrix)
        
    return transformation_matrix
        
def frame_matrix_to_pose(frame_matrix):
    """
    Convert a transfromation matrix into a pose.
    """
        
    # Get translation of frame
    trans = tf.transformations.translation_from_matrix(frame_matrix)
        
    # Get quarternion rotation of the frame
    rot = tf.transformations.quaternion_from_matrix(frame_matrix)
        
    # Create pose from results
    pose_out = Pose()
    pose_out.position.x = trans[0]
    pose_out.position.y = trans[1]
    pose_out.position.z = trans[2]
    pose_out.orientation.x = rot[0]
    pose_out.orientation.y = rot[1]
    pose_out.orientation.z = rot[2]
    pose_out.orientation.w = rot[3]
        
    return pose_out
