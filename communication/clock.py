#!/usr/bin/env python

"""
  Publishes the Gazebo clock message from the simulation computer to 
  another Computer

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation
"""


import sys
import thread

from rosgraph_msgs.msg import Clock
import rospy

from packing import pack_time

# retrive files nessecary for websocket comms
file_dir = sys.path[0]
sys.path.append(file_dir + '/../..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


class SimClockPublisher():
    """ A ROS node to publish the values of the clock t a new computer"""
    
    def __init__(self):
        
        # Get inputs
        self.connection = sys.argv[1]
        
        # Initialize node
        rospy.init_node("clock_publisher", anonymous=True)
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Set the publishing rate (1000 hz)
        self.r = rospy.Rate(1000.0)
        
        # 'robot/joint_state' Subscriber
        rospy.Subscriber("/clock", Clock, self.time_cb)
        
        # Setup ROSbridge publisher
        ws = rC.RosMsg("ws4py", self.connection, "pub", "/clock", "rosgraph_msgs/Clock", pack_time)
        
        # Run publisher
        rospy.loginfo("Clock publisher initialized")
        self.publish_clock(ws)
        
    def time_cb(self, msg):
        """
        Callback for gazebo /clock.
        """
        
        self.lock.acquire()
        self.clock_time = msg
        self.lock.release()
        
    def copy_and_clear_received():
        """
        Clear out received data once retrieved/pulled.
        """
        
        self.lock.acquire()
        clock_current = self.clock_time
        self.clock_time = None
        self.lock.release()
        
        return clock_current
    
    def copy_received():
        """
        just give whatever was last received, don't clear out or overwrite anything.
        """
        
        self.lock.acquire()
        clock_current = self.clock_time
        self.lock.release()
        
        return clock_current
    
    def publish_clock(self, ws):
        """
        Retrieve and publish the clock time.
        """
        
        # Publish the clock to the other computer
        while not rospy.is_shutdown():
            clock_current = self.copy_and_clear_received()
            if (clock_current==None):
                pass
            else:
                ws.send(clock_current)
            self.r.sleep()

    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down 'clock_publisher'")
        rospy.sleep(1)


if __name__ == "__main__":
    SimClockPublisher()
