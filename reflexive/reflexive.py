#!/usr/bin/env python


"""
  Arbotix microcontroller based reflexive layer module for one robotic 
  arm.

  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test pretty much everything
"""


import os
import serial
import sys
import thread
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from csa_msgs.msg import CSADirective, CSAResponse


class ReflexiveModule(object):
    """
    RSE reflexive layer for a dual arm robot.
    
    TODO: Test pretty much everything.
    """
    
    def __init__(self):
        """
        Module initialization
        """
        
        # Get CWD
        home_cwd = os.getcwd()
        
        # Command line inputs
        #TODO
        
        # Retrieve parameters 
        self.rate = rospy.get_param("~rate", 100.0)
        self.side = rospy.get_param("~side", "")
        self.robot = rospy.get_param("~robot", "")
        
        # Setup variables
        self.directives = []
        self.status = ""
        self.description = ""
        self.seq_count = 0
        
        # Initialize rospy node
        rospy.init_node("{}_reflexive_module".format(self.side))
        rospy.loginfo("Node initialized")
        
        # Initialize cleanup for the node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Initialize communications
        self.init_comms()
        rospy.loginfo("Communcation interfaces setup")
        
        # Start state machine
        self.run_state_machine()

    def run_state_machine(self):
        """
        Main execution loop for the module.
        """
        
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.state_machine()
            r.sleep()
    
    def cleanup(self):
        """
        Function to cleanly stop module on shutdown command.
        """
        
        # Shutdown controllers
        for controller in self.controllers:
            controller.shutdown()
        
        # Shutdown node
        rospy.sleep(1)
        rospy.loginfo("Shutting down node")
        
    #==== COMMUNICATIONS ==============================================#
        
    def init_comms(self):
        """
        Initialize the communication interfaces for the module. May use
        ROS patterns or ROSbridge library.
        """
        
        # Message topic names
        response_name = "{}/{}_arm/RL_response".format(self.robot,
                                                       self.side)
        directive_name = "{}/{}_arm/HL_directive".format(self.robot, 
                                                         self.side)
        
        # Setup ROS Publishers
        self.response_pub = rospy.Publisher(response_name, CSAResponse,
                                            queue_size=1)
        
        # Setup ROS Subscribers
        self.directive_sub = rospy.Subscriber(directive_name, CSADirective,
                                              self.directive_cb)
        
    def build_response(self):
        """
        Build a CSA response message
        """
        
        # Create the response message
        msg = CSAResponse()
        
        # Fill out message
        msg.header.seq = self.seq_count
        msg.header.stamp = rospy.Time.now()
        msg.status = self.status
        msg.response = self.description
        
    def directive_cb(self, msg):
        """
        Callback function for directives sent from the habitual message.
        """
        
        # Only store directive if it has new sequence number
        new_seq = msg.header.seq
        if new_seq == self.seq_count:
            pass
        else:
            self.directives.append(msg)
        
    #==== STATE MACHINE ===============================================#
        
    def state_machine(self):
        """
        Perform one update cycle of the module state machine.
        """
        
        pass
        
    #==== CSA SUBMODULES ==============================================#
        
    def arbitration(self):
        """
        Manages the overall behavior of the module: 
            - Comuputes a "merged directive" from all directives recieved 
              from commanding module(s)
            - Issues merged directive to send to the control component
            - Reports status (accept, reject, complete, error,...) to the
              commanding module(s)
        """
        
        pass
        
    def control(self):
        """
        Performs overall function of the module:
            - Computes output directive(s) for commanded modules based on
              directive recieved from the arbitration component
            - Reports success/failure of the directive to the arbitration
              component
        """
        
        pass
        
    def tactics(self):
        """
        Generates a control tactic for control to use based on the merged
        directive.
        """
        
        pass
        
    def activity_manager(self):
        """
        Handles communication with commanded modules.
        """
        
        pass
        

if __name__ == "__main__":
    try:
        module_node = ArbotiXReflexiveModule()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
