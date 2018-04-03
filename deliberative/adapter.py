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
from tf import TransfromerROS, transformations 


#TODO: PASS THESE IN AS ARGUMENTS
REF_FRAME = "base_footprint"


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
        
        # Run script
        self.main()
        
    def compute_eef_offset(self, eef_frames):
        """
        Compute the offset between the two end-effectors rquired to maintain
        relative orientation, in the first arm's frame.
        
        eef_frames: The two end effector frame names, 'lead'-frame first
        
        https://answers.ros.org/question/229329/what-is-the-right-way-to-inverse-a-transform-in-python/
        """
        
        # Create a 'TransfromerROS' object for frame transfromation use
        transformer = tf.TransformerROS()
        
        tf_frames = []
        
        # Convert the two given poses into 'frames' (transformation matrix from origin)
        for frame in eef_frames:
            (trans, rot) = transformer.lookupTransform(frame, REF_FRAME, rospy.Time(0))
            trans_matrix = transformations.translation_matrix(trans)
            rot_matrix = transformations.quaternion_matrix(rot)
            tf_frames.append(transformations.concatenate_matrices(trans_matrix, rot_matrix))
            
        # Invert the lead-frame matrix
        lead_matrix_inverse = transformations.inverse_matrix(tf_frames(0))

        # Get the static transformation matrix from lead to follow frame
        offset = np.dot(lead_matrix_inverse, tf_frame(1))
        
        return offset
        
    def pose_to_frame_matrix(self, pose_stamped):
        """
        Convert a pose into a transformation matrix from the main frame of
        reference.
        """
        
        # Get translation and quaternion rotation out of pose
        trans = pose_stamped.pose.position
        rot = pose_stamped.pose.quaternion
        
        # Convert to 'frame' (transformation matrix from origin)
        trans_matrix = transformations.translation_matrix(trans)
        rot_matrix = transformations.quaternion_matrix(rot)
        frame_matrix = transformations.concatenate_matrices(trans_matrix, rot_matrix)
        
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
        rot = transformations.quaternion_from_matrix(rot_matrix)
        
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
            waypoint.header.position = trans
            waypoint.header.quaternion = rot
            
            # Place into follow pose array
            follow_eef_poses(i) = waypoint
            
        return waypoint
            
    
    def check_trajectories(self, parsed_trajectory, follow_trajectory):
        """
        Check to make sure the trajectories are safe and executable
        """
        pass
        
