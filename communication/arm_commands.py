#!/usr/bin/env python

"""
  For the multicomputer setup, this node publishes the commands 
  for arms on a different computer than the Gazebo simulation. 

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation
"""


import sys
import thread

from std_msgs.msg import Float64
import rospy

from packing import pack_float64

# retrive files nessecary for websocket comms
file_dir = sys.path[0]
sys.path.append(file_dir + '/../..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


class JointCommandsPublisher():
    """
    A ROS node to publish the joint commands of the right WidowX arm to 
    the primary simulation computer.
        
    WARNING: Currently specific to the dual WidowX arm setup I'm running
    
    TODO: More generalized version
    """
    
    def __init__(self):
    
        # Get Inputs
        self.connection = sys.argv[1]
        left_arm = sys.argv[2]
        self.robot = sys.argv[3]
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper_joint"]
        if (left_arm=="True"):
            self.arm = "left"
        elif (left_arm=="False"):
            self.arm = "right"
            
        # Setup storage lists
        self.ws = []
        self.subs = []
        self.commands = [None]*6
        callbacks = [self.joint_1_cb, self.joint_2_cb, self.joint_3_cb, self.joint_4_cb, self.joint_5_cb,self.gripper_joint_cb]
            
        # Initialize node
        rospy.init_node("arm_command_publisher")
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Setup publishing rate (100 Hz)
        self.rate = rospy.Rate(100.0)

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
        """
        Callback for joint_1
        """
        
        # Get the current joint_1 command
        self.lock.acquire()
        self.commands[0] = msg.data
        self.lock.release()
        
    def joint_2_cb(self, msg):
        """
        Callback for joint_2
        """
        
        # Get the current joint_2 command
        self.lock.acquire()
        self.commands[1] = msg.data
        self.lock.release()
        
    def joint_3_cb(self, msg):
        """
        Callback for joint_3
        """
        
        # Get the current joint_3 command
        self.lock.acquire()
        self.commands[2] = msg.data
        self.lock.release()
        
    def joint_4_cb(self, msg):
        """
        Callback for joint_4
        """
        
        # Get the current joint_4 command
        self.lock.acquire()
        self.commands[3] = msg.data
        self.lock.release()
        
    def joint_5_cb(self, msg):
        """
        Callback for joint_5
        """
        
        # Get the current joint_5 command
        self.lock.acquire()
        self.commands[4] = msg.data
        self.lock.release()
        
    def gripper_joint_cb(self, msg):
        """
        Callback for gripper_joint
        """
        
        # Get the current gripper_joint command
        self.lock.acquire()
        self.commands[5] = msg.data
        self.lock.release()
        
    def copy_and_clear_received(self, joint):
        """
        Clear out received data once retrieved/pulled.
        """
        
        # Return the joint command and clear the state variable
        self.lock.acquire()
        command_current = self.command[joint]
        self.command[joint] = None
        self.lock.release()
        
        return command_current
    
    def copy_received(self, joint):
        """
        Give the value that was last received but don't clear or overwrite
        it.
        """
        
        # Return the joint command
        self.lock.acquire()
        command_current = self.command[joint]
        self.lock.release()
        
        return command_current
        
    def publish_joint_commands(self):
        """Retrieve and publish the current joint commands of the arm."""
        
        # Publish the commands only if a new one is recieved
        while not rospy.is_shutdown():
            for i in range(0,len(self.ws)):
                current_command = self.copy_and_clear_received(i)
                if (current_command==None):
                    pass
                else:
                    self.ws[i].send(current_command)
            self.rate.sleep()
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down '{}_joint_command_publisher'".format(self.arm))
        rospy.sleep(1)


if __name__ == "__main__":
    JointCommandsPublisher()

