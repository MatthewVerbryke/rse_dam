#!/usr/bin/env python

"""
  For the multicomputer setup, this node publishes the joints state 
  for arms on a different computer than the Gazebo simulation. 

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation
"""


import sys
import thread

from sensor_msgs.msg import JointState
import rospy

from packing import pack_jointstate

# retrive files nessecary for websocket comms
file_dir = sys.path[0]
sys.path.append(file_dir + '/../..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


class ArmStatePublisher():
    """ 
    A ROS node to publish the joint states of the Boxbot arms to the secondary
    computer.
        
    WARNING: Currently specific to the dual WidowX arm setup I'm running
    
    TODO: More generalized version
    """
    
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
        
        # Storage lists/dicts
        self.header = {}
        self.position = []
        self.velocity = []
        self.effort = []
        
        # Initialize node
        rospy.init_node("{}_state_publisher".format(self.robot), anonymous=True)
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Setup publishing rate (100 Hz)
        self.rate = rospy.Rate(100.0)
        
        # 'robot/joint_state' Subscriber
        rospy.Subscriber("{}/joint_states".format(self.robot), JointState, self.get_joint_states)
        
        # Setup ROSbridge publisher
        ws = rC.RosMsg("ws4py", self.connection, "pub", "{}/joint_states".format(self.robot), "sensor_msgs/JointState", pack_jointstate)
        
        # Run publisher
        rospy.loginfo("Arm state publisher initialized")
        self.publish_arm_state(ws)
        
    def get_joint_states(self, msg):
        """
        Callback for joint states.
        """
        
        # Get the current right arm joint states
        self.lock.acquire()
        self.robot_joint_state = msg
        self.lock.release()
    
    def copy_and_clear_received(self):
        """
        Clear out received data once retrieved/pulled.
        """
        
        # Send the clock time and clear the state variable
        self.lock.acquire()
        arm_state_current = self.robot_joint_state
        self.robot_joint_state = None
        self.lock.release()
        
        return arm_state_current
    
    def copy_received(self):
        """
        Give the value that was last received but don't clear or overwrite
        it.
        """
        
        # Send the clock time
        self.lock.acquire()
        arm_state_current = self.robot_joint_state
        self.lock.release()
        
        return arm_state_current
        
    def publish_arm_state(self, ws):
        """
        Retrieve and publish the current state of the arm.
        """
        
        # Send the states out only if a new joint state message is recieved
        while not rospy.is_shutdown():
            arm_state_current = self.copy_and_clear_received()
            if (arm_state_current==None):
                pass
            else:
                ws.send(arm_state_current)
            self.rate.sleep()

    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down '{}_arm_state_publisher'".format(self.arm))
        rospy.sleep(1)


if __name__ == "__main__":
    ArmStatePublisher()

