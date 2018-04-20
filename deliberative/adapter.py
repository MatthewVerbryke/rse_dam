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

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import numpy as np
import rospy
import tf


#TODO: PASS THESE IN AS ARGUMENTS
REF_FRAME = "base_link"
LEFT_EEF_FRAME = "left_wrist_2_link"
RIGHT_EEF_FRAME = "right_wrist_2_link"
OBJECT_FRAME = 1 #TODO


class RobotTrajectoryAdapter(object):
    """
    A class to adapt trajectories for both arms in a dual-armed robot 
    setup.
    """
    
    def __init__(self):
        """
        Initialize the trajectory adapter.
        """
        
        # Initialize rospy node
        rospy.init_node("dual_arm_trajectory_adapter", anonymous=True)
        rospy.loginfo("Adapter node initialized...")
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Get relevent frame names
        self.ref_frame = REF_FRAME
        self.left_eef_frame = LEFT_EEF_FRAME
        self.right_eef_frame = RIGHT_EEF_FRAME
        
        # Create a transform listener
        self.listener = tf.TransformListener()
        
        # Wait for things to setup properly
        rospy.sleep(1.0)
        
    def compute_eef_offset(self, frame_1, frame_2):
        """
        Compute the offset (transfromation matrix) between the from frame_1
        to frame_2.
        
        Adapted from:
        https://answers.ros.org/question/229329/what-is-the-right-way-to-inverse-a-transform-in-python/
        """
        
        frames = [frame_1, frame_2]
        tf_frames = []
        
        # Get the transformation matrix of each frame from the fixed reference frame
        for frame in frames:
            
            # Try to get the info on the frames from tf
            try:
                (trans, rot) = self.listener.lookupTransform(frame, REF_FRAME, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Error: Failed to recieve the transform for {} to {}".format(REF_FRAME, frame))
                exit() # <--Give it multiple tries instead?
            
            # Determine the transfromation matrix
            trans_matrix = tf.transformations.translation_matrix(trans)
            rot_matrix = tf.transformations.quaternion_matrix(rot)
            tf_frames.append(tf.transformations.concatenate_matrices(trans_matrix, rot_matrix))
            
        # Invert the  first frame matrix
        matrix_1_inverse = tf.transformations.inverse_matrix(tf_frames[0])
        
        # Get the static transformation matrix from lead to follow frame
        offset = np.dot(matrix_1_inverse, tf_frames[1])
        
        return offset
        
    def pose_to_frame_matrix(self, pose_stamped):
        """
        Convert a pose into a transformation matrix
        
        TODO: TEST THIS FUNCTION
        """
        
        # Get translation and quaternion rotation out of pose
        trans = pose_stamped.pose.position
        rot = pose_stamped.pose.orientation
        
        # Convert to 'frame' (transformation matrix from origin)
        trans_matrix = tf.transformations.translation_matrix(trans)
        rot_matrix = tf.transformations.quaternion_matrix(rot)
        transformation_matrix = tf.transformations.concatenate_matrices(trans_matrix, rot_matrix)
        
        return transformation_matrix
        
    def frame_matrix_to_pose(self, frame_matrix):
        """
        Convert a transfromation matrix into a pose.
        
        TODO: TEST THIS FUNCTION
        """
        
        # Get translation of frame
        trans = transformations.translation_from_matrix(frame_matrix)
        
        # Get quarternion rotation of the frame
        #   Not sure about this, might need to back out only rot part of matrix
        #   TODO: test this stuff first, to make sure its working
        print(frame_matrix)
        rot_matrix = frame_matrix[0:3,0:3]
        rot = tf.transformations.quaternion_from_matrix(rot_matrix)
        
        # Create pose from results
        pose_out = Pose()
        pose_out.positon = trans
        pose_out.orientation = rot
        
        return pose_out
        
    def adapt_arm_poses(self, time_stamps, object_poses)
        """
        Create an array of poses for each of the arms based on the object
        pose array, each using their respective offset. 
        
        TODO: TEST THIS FUNCTION
        """
        
        # Setup resultant PoseArray's
        left_poses = PoseArray()
        left_poses.header = object_poses.header
        right_poses = PoseArray()
        right_poses.header = object_poses.header
        
        # Get the offsets for the left and right eef's
        left_offset = self.compute_eef_offset(object_frame, left_eef_frame)
        right_offset = self.compute_eef_offset(object_frame, right_eef_frame)
        
        # Setup loop
        traj_wp_num = len(object_poses.poses)
        left_pose_list = [None]*traj_wp_num
        right_pose_list = [None]*traj_wp_num
        
        # Determine eef poses for each object pose
        for i in range(0,traj_wp_num):
            
            # Get object transform matrix at the time step
            object_matrix = self.pose_to_frame_matrix(object_poses[i])
                        
            # Determine the left eef pose at the time step
            left_eef_matrix = np.dot(object_matrix, left_offset)
            left_pose = frame_matrix_to_pose(left_eef_matrix)
            
            # Determine the right eef pose at the time step
            right_eef_matrix = np.dot(object_matrix, right_offset)
            right_pose = frame_matrix_to_pose(right_eef_matrix)
            
            # Store results
            left_pose_list[i] = left_pose
            right_pose_list[i] = right_pose
            
        # Store final arrays in PoseArray's
        left_poses.poses = left_pose_list
        right_poses.poses = right_pose_list
        
        return left_poses, right_poses
    
    def get_velocities(self, traj):
        """
        TODO
        """
        pass
    
    def check_trajectories(self, parsed_trajectory, follow_trajectory):
        """
        TODO
        """
        pass
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down node 'dual_arm_trajectory_adapter'")
        rospy.sleep(1)
        
