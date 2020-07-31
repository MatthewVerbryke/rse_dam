#!/usr/bin/env python

"""
  Servo interface objects for use with the ArbotiX reflexive layer. 
  Mostly taken from: 
  https://github.com/MatthewVerbryke/arbotix_ros/blob/turtlebot2i/arbotix_python/src/arbotix_python/servo_controller.py
 
  Copyright 2011-2013 Vanadium Labs LLC.
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test pretty much everything
"""


import rospy

from ax12 import *


class DynamixelInterface(object):
    """
    
    """
    
    def __init__(self, device, name):
        
        # Input parameters
        self.name = name
        self.device = device
        
        # Get joint names
        self.joints = rospy.get_param("~controllers/arm_controllers/joints")
        self.iter = 0
        
        # Get read and write rate parameters
        self.w_delta = rospy.Duration(1.0/rospy.get_param("~write_rate", 10.0))
        self.w_next = rospy.Time.now() + self.w_delta
        self.r_delta = rospy.Duration(1.0/rospy.get_param("~read_rate", 10.0))
        self.r_next = rospy.Time.now() + self.r_delta
        
        # Create synclist of dynamixel ID's
        synclist = list()
        for joint in self.dynamixels:
            synclist.append(joint.id)
            
    def update(self):
        """
        Read servo positions, update them.
        """
        
        # Read data from Dynamixels
        if rospy.Time.now() > self.r_next:
            pos = self.device.syncRead(synclist, P_PRESENT_POSITION_L, 2)
            dgn = self.device.syncRead(synclist, P_PRESENT_VOLTAGE, 2)
            load = self.device.syncRead(synclist, P_PRESENT_LOAD_L, 2)
            
            # Retrive info from raw data
            for joint in self.dynamixels:
                try:
                    
                    # Retrieve joint position
                    i = synclist.index(joint.id)*2 
                    joint.set_current_feedback(pos[i]+(pos[i+1]<<8))
                
                    # Retrieve diagnostic data (every fifth iteration)
                    if self.iter % 5 == 0:
                        if val[i] < 250:
                            joint.voltage = dgn[i]/10.0
                        if val[i+1] < 100:
                            joint.temperature = dgn[i+1]
                        
                    # Retrieve joint loads
                    joint.load = float(((load[i]+(load[i+1]<<8))-1024)*0.1)
                    
                except Exception as e:
                    rospy.logerr("Servo read error: " + str(e) )
                    joint.load = 1024
                    continue
            
            # Get next read time
            self.r_next = rospy.Time.now() + self.r_delta
    
        # Write commands to the Dynamixels
        if rospy.Time.now() > self.w_next:
            
            # Create list of new commands
            syncpkt = list()
            for joint in self.dynamixels:
                v = joint.interpolate(1.0/self.w_delta.to_sec())
                if v != None:
                    syncpkt.append([joint.id,int(v)%256,int(v)>>8])
                
            # If we have commands sync-write them
            if len(syncpkt) > 0:      
                self.device.syncWrite(P_GOAL_POSITION_L,syncpkt)

            # Get next write time
            self.w_next = rospy.Time.now() + self.w_delta
            self.iter += 1
            
        return none
