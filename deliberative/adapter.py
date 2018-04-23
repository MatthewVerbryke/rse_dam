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

import trajectories


#TODO: PASS THESE IN AS ARGUMENTS
REF_FRAME = "base_link"
LEFT_EEF_FRAME = "left_wrist_2_link"
RIGHT_EEF_FRAME = "right_wrist_2_link"
OBJECT_FRAME = 1 #TODO
SPEED = 0.05 #m/s
LIFT_HEIGHT = 0.1

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
        self.speed = SPEED
        self.lift_height = LIFT_HEIGHT
        
        # Create a transform listener
        self.listener = tf.TransformListener()
        
        # Wait for things to setup properly
        rospy.sleep(1.0)
        
    def get_object_trajectory(self, template):
        """
        Based on the selected trajectory plan (user specified?), get a 
        trajectory that the target object must follow.
        """
        
        # Create the object trajectory based on the template choice
        if (template=="simple pick and place"):
            trajectory = create_simple_move_trajectory(start_pose, goal_pose, self.lift_height, self.speed)
        else:
            print "Error: trajectory type not found"
            
        return trajectory
        
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
        
    def adapt_arm_poses(self, time_stamps, object_poses, offset, side):
        """
        Create an array of poses for the arm based on the object pose array,
        using the predetermined offset. 
        
        TODO: TEST THIS FUNCTION
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
            object_matrix = self.pose_to_frame_matrix(object_poses[i])
            
            # Determine the eef pose at the time step
            eef_matrix = np.dot(object_matrix, offset)
            pose_i = frame_matrix_to_pose(eef_matrix)
            
            # Store results
            pose_list[i] = pose_i
            
        # Store final arrays in PoseArray's
        poses.poses = pose_list
        
        return poses
    
    def check_trajectories(self, parsed_trajectory, follow_trajectory):
        """
        TODO
        """
        pass
        
    def pose_to_frame_matrix(self, pose_in):
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
        trans_matrix = tf.transformations.translation_matrix(trans);
        rot_matrix = tf.transformations.quaternion_matrix(rot);
        transformation_matrix = tf.transformations.concatenate_matrices(trans_matrix, rot_matrix)
        
        return transformation_matrix
        
    def frame_matrix_to_pose(self, frame_matrix):
        """
        Convert a transfromation matrix into a pose.
        
        NOTE: quaternions may not be functioning correctly; needs more tests
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
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        # Log shutdown
        rospy.loginfo("Shutting down node 'dual_arm_trajectory_adapter'")
        rospy.sleep(1)
