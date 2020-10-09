#!/usr/bin/env python

"""
  A solver which solves for the current_state of each end-effector in a
  dual-arm robot.
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test
"""


from geometry_msgs.msg import Pose
import rospy

import tf


class EndEffectorStateSolver(object):
    """
    A dual-arm end-effector state solver object
    """
    
    def __init__(self, robot, arms):
        
        # Parameters
        self.robot = robot
        self.base_frame = ""
        self.left_base = arms["left_arm"]["base"]
        self.left_eef = arms["left_arm"]["eef"]
        self.right_base = arms["right_arm"]["base"]
        self.right_eef = arms["right_arm"]["eef"]

        # Create a TF listener
        self.listener = tf.TransformListener()
    
    def lookup_frame_transform(self, origin, target):       
        """
        Get the transform (translation and quaternion rotation) between 
        two frames using tf.
        """   
        
        # Try to get the info on the frames from tf if given a link name
        try:
            self.listener.waitForTransform(target, origin, rospy.Time(0), rospy.Duration(4.0))
            trans, rot = self.listener.lookupTransform(target, origin, rospy.Time(0))
            return [trans, rot]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to recieve the transform for {} to {}".format(origin, target))
            return [None, None]
    
    def update(self, joint_state):
        """
        Update the function when called from the main script.
        """
        
        # Get current transforms for the end-effector
        trans_left_eef = self.lookup_frame_transform(self.base_frame, self.left_eef)
        trans_right_eef = self.lookup_frame_transform(self.base_frame, self.right_eef)
        
        # Package the transforms as geometry_msgs/Pose
        left_eef_pose = Pose()
        left_eef_pose.position = trans_left_eef[0]
        left_eef_pose.orientation = trans_left_eef[1]
        right_eef_pose = Pose()
        right_eef_pose.position = trans_right_eef[0]
        right_eef_pose.orientation = trans_right_eef[1]
        
        return [left_eef_pose, right_eef_pose], "ok"
