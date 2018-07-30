#!/usr/bin/env python

"""
  Functions to parse .bag files for data specific to rse_dam scenarios
  
  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.
"""


import math
import sys

import rosbag
import matplotlib.pyplot as plt


class RSESimBagParser(object):
    """
    Parse bag file data and prepare for figure printing for sim data.
    """
    
    def __init__(self):
        
        # Open the bag file
        bag_name = sys.argv[1]
        self.bag = rosbag.Bag(str(bag_name))
        
        # Get what data needs to be graphed
        self.graph_clock = False
        self.graph_eef_pose = True
        self.graph_contacts = True
        self.graph_block = True
        
        # Initialize holding lists
        self.state_time = []
        self.system_time = []
        self.left_contact_time = []
        self.right_contact_time = []
        self.contact_time = []
        self.clock_time = []
        self.left_eef_state = []
        self.right_eef_state = []
        self.left_force = []
        self.left_torque = []
        self.right_force = []
        self.right_torque = []
        self.block_force = []
        self.block_torque = []
        
        # Run
        self.main()

    def parse_clock (self):
        """
        Parse the .bag file for the gazebo clock time.
        """
        
        # Parse the 'clock' topic to get the relation between system time and clock time
        for topic, msg, t in self.bag.read_messages(topics=['/clock']):
            
            # Convert stamp into float time
            time = self.get_float_time(t)
            self.system_time.append(time)
            
            # Parse the clock time 
            cl_time = self.get_float_time(msg.clock)
            self.clock_time.append(cl_time)
        
    def parse_eef_states(self):
        """
        Parse the .bag file for end effector state information data.
        """
        # Parse the '/link_states' topic to get the desired link states out
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
                self.left_torque.append(0.0)
            else:
                for contact in msg.states:
                    total_force = []
                    total_torque = [] 
                    total_force.append(math.sqrt((contact.total_wrench.force.x)**2
                                   + (contact.total_wrench.force.y)**2
                                   + (contact.total_wrench.force.z)**2))
                    total_torque.append(math.sqrt((contact.total_wrench.torque.x)**2
                                   + (contact.total_wrench.torque.y)**2
                                   + (contact.total_wrench.torque.z)**2))
                self.left_force.append(sum(total_force))
                self.left_torque.append(sum(total_torque))
                    
        # Parse right contact states topic to get contact force data out
        for topic, msg, t in self.bag.read_messages(topics=['/right_test_block_contact_state']):

            # Convert stamp into float time
            r_time = self.get_float_time(msg.header.stamp)
            self.right_contact_time.append(r_time)
            
            # Parse the message for force info
            if (msg.states==[]):
                self.right_force.append(0.0)
                self.right_torque.append(0.0)
            else:
                total_force = []
                total_torque = [] 
                for contact in msg.states:
                    total_force.append(math.sqrt((contact.total_wrench.force.x)**2
                                   + (contact.total_wrench.force.y)**2
                                   + (contact.total_wrench.force.z)**2))
                    total_torque.append(math.sqrt((contact.total_wrench.torque.x)**2
                                   + (contact.total_wrench.torque.y)**2
                                   + (contact.total_wrench.torque.z)**2))
                self.right_force.append(sum(total_force))
                self.right_torque.append(sum(total_torque))
    
    def parse_block_contact_states(self):
        
        # Parse right contact states topic to get contact force data out
        for topic, msg, t in self.bag.read_messages(topics=['/block_contact_state']):
            
            # Convert stamp into float time
            r_time = self.get_float_time(msg.header.stamp)
            self.contact_time.append(r_time)
            
            # Parse the message for force info
            if (msg.states==[]):
                self.block_force.append(0.0)
                self.block_torque.append(0.0)
            else:
                for contact in msg.states:
                    total_force = []
                    total_torque = [] 
                    total_force.append(math.sqrt((contact.total_wrench.force.x)**2
                                   + (contact.total_wrench.force.y)**2
                                   + (contact.total_wrench.force.z)**2))
                    total_torque.append(math.sqrt((contact.total_wrench.torque.x)**2
                                   + (contact.total_wrench.torque.y)**2
                                   + (contact.total_wrench.torque.z)**2))
                self.block_force.append(sum(total_force))
                self.block_torque.append(sum(total_torque))
    
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
        
    def plot_poses(self, side):
        """
        Plot the poses of the end-effectors
        """
        
        t = []
        x = []
        y = []
        z = []
        qx = []
        qy = []
        qz = []
        qw = []
        
        # Get times
        for time in self.state_time:
            t.append(time)
        
        # Determine pose components
        if (side=="left"):
            for pose in self.left_eef_state:
                x.append(pose.position.x)
                y.append(pose.position.y)
                z.append(pose.position.z)
                qx.append(pose.orientation.x)
                qy.append(pose.orientation.y)
                qz.append(pose.orientation.z)
                qw.append(pose.orientation.w)
        elif (side=="right"):
            for pose in self.right_eef_state:
                x.append(pose.position.x)
                y.append(pose.position.y)
                z.append(pose.position.z)
                qx.append(pose.orientation.x)
                qy.append(pose.orientation.y)
                qz.append(pose.orientation.z)
                qw.append(pose.orientation.w)
                
        # Plot the results
        plt.plot(t, x, c='r', ls='-')
        plt.plot(t, y, c='g', ls='-')
        plt.plot(t, z, c='b', ls='-')
        plt.show()
        
    def plot_forces(self):
        """
        Plot forces on the test blocks.
        """
        
        # Plot forces
        plt.plot(self.right_contact_time, self.right_force, c='c', ls='-')
        plt.plot(self.left_contact_time, self.left_force, c='m', ls='-')
        plt.plot(self.contact_time, self.left_force, c='c', ls='-')
        
        # Plot torques
        plt.plot(self.right_contact_time, self.right_torque, c='c', ls='-')
        plt.plot(self.left_contact_time, self.left_torque, c='m', ls='-')
        plt.show()
        
    def plot_block_forces(self):
        """
        Plot forces on the 'loose' test block
        """
        
        # Plot forces and torques
        plt.plot(self.contact_time, self.left_force, c='c', ls='-')
        plt.plot(self.contact_time, self.left_torque, c='c', ls='--')
        plt.show()
        
    def main(self):
        """
        Main execution loop
        """
        
        # Parse topic data
        self.parse_clock()
        self.parse_eef_states()
        self.parse_block_contact_states()
        
        # Check clock to message time difference
        #for time in range(0,len(self.system_time)):
            #diff = self.system_time[time] - self.clock_time[time]
            #print diff

        # See if clock time and recorded time match up
        if self.graph_clock:
            self.plot_graphs(self.system_time, self.clock_time)
        
        # Plot the end-effector pose histories
        if self.graph_eef_pose:
            self.plot_poses("left")
            self.plot_poses("right")
        
        # Plot the loose block forces and position
        if self.graph_block:
            self.plot_block_forces()
        
        # Plot the test-block forces
        if self.graph_contacts:
            self.plot_forces()
            
        self.bag.close()
    
    def close_bag(self):
        """
        Close the bag file.
        """
        self.bag.close()


if __name__ == "__main__":
    RSESimBagParser()
