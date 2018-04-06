#!/usr/bin/env python

"""
  TODO: module description

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
"""


from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rospy
import tf


#TODO: PASS THESE IN AS ARGUMENTS
REF_FRAME = "base_link"


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
        print offset
        return offset
        
    def pose_to_frame_matrix(self, pose_stamped):
        """
        Convert a pose into a transformation matrix from the main frame of
        reference.
        """
        
        # Get translation and quaternion rotation out of pose
        trans = pose_stamped.pose.position
        rot = pose_stamped.pose.orientation
        
        # Convert to 'frame' (transformation matrix from origin)
        trans_matrix = tf.transformations.translation_matrix(trans)
        rot_matrix = tf.transformations.quaternion_matrix(rot)
        frame_matrix = tf.transformations.concatenate_matrices(trans_matrix, rot_matrix)
        
        return frame_matrix
        
    def frame_matrix_to_pose(self, frame_matrix):
        """
        Convert a transfromation matrix from the main frame of reference
        into a pose.
        """
        
        # Get translation of frame
        trans = transformations.translation_from_matrix(frame_matrix)
        
        # Get quarternion rotation of the frame
        #   Not sure about this, might need to back out only rot part of matrix
        #   TODO: test this stuff first, to make sure its working
        print(frame_matrix)
        rot_matrix = frame_matrix[0:3,0:3]
        rot = tf.transformations.quaternion_from_matrix(rot_matrix)
        
        return trans, rot
        
    def adapt_follow_poses(self, time_stamps, lead_eef_poses, offset):
        """
        Create an array of poses for the 'follow' arm that are at the required
        fixed offset from the 'lead arm at each time_step.
        """
        
        # Loop setup
        waypoint_tot = len(lead_eff_poses)
        follow_eef_poses = [None]*waypoint_tot
        
        # Build array of 'PoseStamped' waypoints
        for i in range(0,waypoint_tot):
            
            # Determine and decompose follow frame matrix
            lead_frame_matrix = self.pose_to_frame_matrix(lead_eef_poses(i))
            follow_frame_matrix = np.dot(lead_frame_matrix, offset);
            trans, rot = frame_matrix_to_pose(follow_frame_matrix)
            print (trans, rot)
            
            # Construct 'PoseStamped'
            waypoint = PoseStamped()
            waypoint.header.frame_id = REF_FRAME
            waypoint.header.stamp = time_stamp(i)
            waypoint.pose.position = trans
            waypoint.pose.orientation = rot
            
            # Place into follow pose array
            follow_eef_poses[i] = waypoint
            
        return follow_eef_poses
            
    
    def check_trajectories(self, parsed_trajectory, follow_trajectory):
        """
        Check to make sure the trajectories are safe and executable
        """
        pass
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down node 'dual_arm_trajectory_adapter'")
        rospy.sleep(1)
        
