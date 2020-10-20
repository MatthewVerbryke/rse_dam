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
        self.base_frame = "torso" #<-- TODO: change this to be more general
        self.left_base = arms["left_arm"]["base"]
        self.left_eef = arms["left_arm"]["eef"]
        self.right_base = arms["right_arm"]["base"]
        self.right_eef = arms["right_arm"]["eef"]
        print self.left_eef, self.right_eef

        # Create a TF listener
        self.listener = tf.TransformListener()
    
    def lookup_frame_transform(self, origin, target):       
        """
        Get the transform (translation and quaternion rotation) between 
        two frames using tf.
        """   
        
        # Try to get the info on the frames from tf if given a link name
        try:
            self.listener.waitForTransform(origin, target, rospy.Time(), rospy.Duration(1.0))
            trans, rot = self.listener.lookupTransform(origin, target, rospy.Time())
            return [trans, rot]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to recieve the transform for {} to {}".format(origin, target))
            return [None, None]
    
    def tf_transform_to_pose(self, transform):
        """
        Given a transform from 'tf/TransformListener' convert it into a
        'geometry_msgs/Pose' message.
        """
        
        pose = Pose()
        pose.position.x = transform[0][0]
        pose.position.y = transform[0][1]
        pose.position.z = transform[0][2]
        pose.orientation.x = transform[1][0]
        pose.orientation.y = transform[1][1]
        pose.orientation.z = transform[1][2]
        pose.orientation.w = transform[1][3]
        
        return pose
    
    def update(self, joint_state):
        """
        Update the function when called from the main script.
        """
        
        # Get current transforms for the end-effector
        trans_left_eef = self.lookup_frame_transform(self.base_frame, self.left_eef)
        trans_right_eef = self.lookup_frame_transform(self.base_frame, self.right_eef)
        
        # Package the transforms as geometry_msgs/Pose
        left_eef_pose = self.tf_transform_to_pose(trans_left_eef)
        right_eef_pose = self.tf_transform_to_pose(trans_right_eef)
        
        return [left_eef_pose, right_eef_pose], "ok"
