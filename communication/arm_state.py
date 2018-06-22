#!/usr/bin/env python

"""
  For the multicomputer setup, this node publishes the joints state 
  for arms on a different computer than the Gazebo simulation. 

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
  
  TODO: TEST THIS
"""

import sys 

from sensor_msgs.msg import JointState
import rospy

from packing import pack_jointstate

# retrive files nessecary for websocket comms
file_dir = sys.path[0]
sys.path.append(file_dir + '/../..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


# Pass in as arguments
ROBOT = "boxbot"
CONNECTION = "ws://192.168.0.51:9090/" #<--change!
ARM = "right"

class ArmStatePublisher():
    """  """
    
    def __init__(self):
        
        # Get inputs
        self.connection = sys.argv[1]
        left_arm = sys.argv[2]
        self.joint_names = ["right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_gripper_joint"]
        self.robot = sys.argv[3]
        if (left_arm=="True"):
            self.arm = "left"
        elif (left_arm=="False"):
            self.arm = "right"
        
        self.header = {}
        self.position = []
        self.velocity = []
        self.effort = []
        
        # Initialize node
        rospy.init_node("{}_state_publisher".format(self.robot), anonymous=True)
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # 'robot/joint_state' Subscriber
        rospy.Subscriber("{}/joint_states".format(self.robot), JointState, self.get_joint_states)
        
        # Setup ROSbridge publisher
        ws = rC.RosMsg("ws4py", self.connection, "pub", "{}/joint_states".format(self.robot), "sensor_msgs/JointState", pack_jointstate)
        
        # Setup timessteps
        self.w_delta = rospy.Duration(1.0/rospy.get_param("~write_rate", 10.0))
        self.w_next = rospy.Time.now() + self.w_delta
        self.r_delta = rospy.Duration(1.0/rospy.get_param("~read_rate", 10.0))
        self.r_next = rospy.Time.now() + self.r_delta
        
        # Run publisher
        rospy.loginfo("Arm state publisher initialized")
        self.publish_arm_state(ws)
        
    def get_joint_states(self, msg):
        """
        Callback for joint states.
        """
        self.robot_joint_state = msg
        
    def publish_arm_state(self, ws):
        """
        Retrieve and publish the current state of the arm.
        """
        
        while not rospy.is_shutdown():
            
            # Send the states out at the appropriate time
            if rospy.Time.now() > self.r_next:
                ws.send(self.robot_joint_state)
                rospy.sleep(0.01)

    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down '{}_arm_state_publisher'".format(self.arm))
        rospy.sleep(1)


if __name__ == "__main__":
    ArmStatePublisher()

