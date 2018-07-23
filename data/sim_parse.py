#!/usr/bin/env python

"""
  Functions to parse .bag files for data specific to rse_dam scenarios
  
  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.
"""


import sys

import rosbag


class RSESimBagParser:
    """
    Parse bag file data and prepare for figure printing for sim data.
    """
    
    def __init__(self):
        
        # Open the bag file
        bag_name = sys.argv[1]
        self.bag = rosbag.Bag(str(bag_name))
        
        # Initialize holding lists
        self.state_time = []
        self.left_contact_time = []
        self.right_contact_time = []
        self.left_eef_state = []
        self.right_eef_state = []
        self.left_force = []
        self.left_torque = []
        self.right_force = []
        self.right_torque = []

    def parse_eef_states(self):
        """
        Parse the .bag file for end effector state information data.
        """
        
        # Parse '/link_states' topic to get the desired link states out
        for topic, msg, t in self.bag.read_messages(topics=['/gazebo/link_states']):
            
            # Convert stamp into float time
            time = self.get_float_time(t)
            self.state_time.append(time) 
            
            # Parse eef_states
            left_index = msg.name.index("boxbot::left_wrist_2_link")
            self.left_eef_state.append(msg.pose[left_index])
            right_index = msg.name.index("boxbot::right_wrist_2_link")
            self.right_eef_state.append(msg.pose[right_index])
    
    def parse_contact_states(self):
        """
        Parse the .bag file for contact state data
        """
        
        # Parse right contact states topic to get contact force data out
        for topic, msg, t in self.bag.read_messages(topics=['/left_test_block_contact_state']):

            # Convert stamp into float time
            r_time = self.get_float_time(msg.header.stamp)
            self.left_contact_time.append(r_time)
            
            # Parse the message for force info
            if (msg.states==[]):
                self.left_force.append(0.0)
            else:
                for contact in msg.states:
                    self.left_force.append(contact.total_wrench.force)
                    self.left_torque.append(contact.total_wrench.torque)
                    
        # Parse right contact states topic to get contact force data out
        for topic, msg, t in self.bag.read_messages(topics=['/right_test_block_contact_state']):

            # Convert stamp into float time
            r_time = self.get_float_time(msg.header.stamp)
            self.right_contact_time.append(r_time)
            
            # Parse the message for force info
            if (msg.states==[]):
                self.right_force.append(0.0)
            else:
                for contact in msg.states:
                    self.right_force.append(contact.total_wrench.force)
                    self.right_torque.append(contact.total_wrench.torque)
        
    def get_float_time(self, stamp):
        """
        Convert a stamp style representation of time (sec, nsec) into a floating-
        point representation.
        """
        
        # Convert nsec from nsecs (int) to seconds (float)
        float_nsecs = float(stamp.nsecs)*0.000000001
        
        # Add the two components together
        float_secs = float(stamp.secs) + float_nsecs

        return float_secs
    
    def close_bag(self):
        """
        Close the bag file.
        """
        self.bag.close()


if __name__ == "__main__":
    try:
        test = RSEBagParser()
        test.parse_eef_states()
        test.parse_contact_states()
        test.close_bag()
    except:
        pass
