#!/usr/bin/env python

"""
  For the multicomputer setup, this node publishes the commands 
  for arms on a different computer than the Gazebo simulation. 

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
"""

import sys 

from std_msgs.msg import Float64
import rospy

from packing import pack_float64

# retrive files nessecary for websocket comms
file_dir = sys.path[0]
sys.path.append(file_dir + '/../..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


# Pass in as arguments
ROBOT = "boxbot"
CONNECTION = "ws://192.168.0.51:9090/" #<--change!
ARM = "right"

class JointCommandsPublisher():
    """ WARN: Robot specific and very crude/WIP"""
    
    def __init__(self):
    
        # Get Inputs
        self.connection = sys.argv[1]
        left_arm = sys.argv[2]
        self.robot = sys.argv[3]
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper_joint"]
        self.commands = [None]*6
        if (left_arm=="True"):
            self.arm = "left"
        elif (left_arm=="False"):
            self.arm = "right"
            
        # Initialize node
        rospy.init_node("arm_command_publisher")
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Setup timessteps
        self.w_delta = rospy.Duration(1.0/rospy.get_param("~write_rate", 10.0))
        self.w_next = rospy.Time.now() + self.w_delta
        self.r_delta = rospy.Duration(1.0/rospy.get_param("~read_rate", 10.0))
        self.r_next = rospy.Time.now() + self.r_delta
        self.r = rospy.Rate(10.0)

        self.ws = []
        self.subs = []
        
        callbacks = [self.joint_1_cb, self.joint_2_cb, self.joint_3_cb, self.joint_4_cb, self.joint_5_cb,self.gripper_joint_cb]
        
        # Setup ROSbridge publishers and ROSsubscribers
        for i in range(0,len(self.joint_names)):
            topic = self.robot + "/" + self.arm + "_" + self.joint_names[i] + "_controller/command"
            self.subs.append(rospy.Subscriber(topic, Float64, callbacks[i]))
            self.ws.append(rC.RosMsg("ws4py", self.connection, "pub", topic, "std_msgs/Float64", pack_float64))
        
        # Run publishers
        rospy.sleep(5)
        rospy.loginfo("Joint command publisher initialized")
        self.publish_joint_commands()
      
    def joint_1_cb(self, msg):
        """ Callback for joint 1 """
        self.commands[0] = msg.data
        
    def joint_2_cb(self, msg):
        """ Callback for joint 2 """
        self.commands[1] = msg.data
        
    def joint_3_cb(self, msg):
        """ Callback for joint 1 """
        self.commands[2] = msg.data
        
    def joint_4_cb(self, msg):
        """ Callback for joint 1 """
        self.commands[3] = msg.data
        
    def joint_5_cb(self, msg):
        """ Callback for joint 1 """
        self.commands[4] = msg.data
        
    def gripper_joint_cb(self, msg):
        """ Callback for joint 1 """
        self.commands[5] = msg.data
        
    def publish_joint_commands(self):
        """Retrieve and publish the current state of the arm."""
        
        while not rospy.is_shutdown():
            
            # Publish the commands
            if rospy.Time.now() > self.r_next:
                for i in range(0,len(self.ws)):
                    self.ws[i].send(self.commands[i])
            self.r.sleep()
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down '{}_joint_command_publisher'".format(self.arm))
        rospy.sleep(1)

if __name__ == "__main__":
    JointCommandsPublisher()

