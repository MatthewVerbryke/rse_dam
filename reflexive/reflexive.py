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
from arbotix_python.arbotix import ArbotiX
from diagnostic_msgs.msg import DiagnosticArray
from servos import DynamixelServo
from interfaces import DynamixelInterface
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from csa_msgs.msg import CSADirective, CSAResponse


class ArbotiXReflexiveModule(object):
    """
    RSE reflexive layer for a dual arm robot using an Arbotix type micro-
    controller.
    
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
        port = rospy.get_param("~port", "/dev/arbotix")
        baud = int(rospy.get_param("~baud", "115200"))
        self.rate = rospy.get_param("~rate", 100.0)
        self.sim = rospy.get_param("~sim", False)
        self.side = rospy.get_param("~side", "left")
        
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
        
        # Setup Arbotix hardware driver if not in simulation
        if not self.sim:
            self.driver = ArbotiX.__init__(self, port, baud)
            rospy.sleep(1.0)
            rospy.loginfo("Started ArbotiX connection on port " + port + ".")
            self.setup_hw_interface()
        else:
            # TODO
            rospy.loginfo("ArbotiX being simulated.")
        
        # Initialize communications
        self.init_comms()
        rospy.loginfo("Communcation interfaces setup")
        
        # Start state machine
        self.run_state_machine()
        
    def setup_hw_interface(self):
        """
        Setup the required interfaces for communication with the hardware
        """
        
        # Setup joints
        self.driver.joints = dict{}
        for name in rospy.get_param("~joints", dict()).keys()
            self.driver.joints[name] = DynamixelServo(name)
        
        # Setup controllers
        self.driver.controllers = [DynamixelInterface(self.driver, "servos"), ]
        controllers = rospy.get_param("~controllers", dict())
        for name, params in controllers.items():
            try:
                controller = controller_types[params["type"]](self.driver, name)
                self.driver.controllers.append(controller)
                pause = pause or controller.pause
            except Exception as e:
                if type(e) == KeyError:
                    rospy.logerr("Unrecognized controller: " + params["type"])
                else:  
                    rospy.logerr(str(type(e)) + str(e))
        
        # Start controllers
        for controller in self.driver.controllers:
            controller.startup()

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
        joint_state_name = "{}_joint_states".format(self.side)
        diagnostic_name = "{}_diagnostics".format(self.side)
        response_name = "{}_arm/RL_response/".format(self.side)
        directive_name = "{}_arm/HL_directive".format(self.side)
        
        # Setup ROS Publishers
        self.joint_state_pub = rospy.Publisher(joint_state_name, JointState,
                                               queue_size=5)
        self.diagnostic_pub = rospy.Publisher(diagnostic_name, DiagnosticArray,
                                              queue_size=5)
        self.response_pub = rospy.Publisher(response_name, CSAResponse,
                                            queue_size=1)
        
        # Setup ROS Subscribers
        self.directive_sub = rospy.Subscriber(directive_name, CSADirective,
                                              self.directive_cb)
    
    def build_diagnostic_msg(self):
        """
        Build a DiagnosticArray message from current data.
        """
        
        # Create message
        msg = DiagnosticArray()
        
        # Fill out message
        msg.header.stamp = rospy.Time.now()
        for joint in self.driver.joints:
            d = joint.get_diagnostics()
            if d:
                msg.status.append(d)
        
        return msg
        
    def build_joint_state_msg(self):
        """
        Build a JointState message from the currently up-to-date joint 
        states.
        """
        
        # Create message
        msg = JointState()
        
        # Fill out message
        msg.header.stamp() = rospy.Time.now()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = [] # <-- Using this to pub the joint loads
        for joint in self.driver.joints:
            msg.name.append(joint.name)
            msg.position.append(joint.jst_position)
            msg.velocity.append(joint.velocity)
            msg.effort.append(joint.load)
            
        return msg
        
    def build_response(self):
        """
        Buiild a CSA response message
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
