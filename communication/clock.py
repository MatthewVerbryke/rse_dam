#!/usr/bin/env python

"""
  Publishes the Gazebo clock message from the simulation computer to 
  another Computer

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board

"""

import sys

from rosgraph_msgs.msg import Clock
import rospy

from packing import pack_time

# retrive files nessecary for websocket comms
file_dir = sys.path[0]
sys.path.append(file_dir + '/../..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


# Pass in as arguments
CONNECTION = "ws://192.168.0.51:9090/" #<--change!

class SimClockPublisher():
    """  """
    
    def __init__(self):
        
        # Get inputs
        self.connection = sys.argv[1]
        
        # Initialize node
        rospy.init_node("clock_publisher", anonymous=True)
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # 'robot/joint_state' Subscriber
        rospy.Subscriber("/clock", Clock, self.time_cb)
        
        # Setup ROSbridge publisher
        ws = rC.RosMsg("ws4py", self.connection, "pub", "/clock", "rosgraph_msgs/Clock", pack_time)
        
        # Run publisher
        rospy.loginfo("Clock initialized")
        self.publish_clock(ws)
        
    def time_cb(self, msg):
        """
        Callback for gazebo clock.
        """
        self.clock_time = msg
        
    def publish_clock(self, ws):
        """
        Retrieve and publish the clock time.
        """
        
        # Publish the clock to the other computer
        while not rospy.is_shutdown():
            ws.send(self.clock_time)
            rospy.sleep(0.001)

    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down 'clock_publisher'")
        rospy.sleep(1)


if __name__ == "__main__":
    SimClockPublisher()
