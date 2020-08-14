#!/usr/bin/env python

"""
  Main reflexive-layer module for one robotic arm in a dual-arm setup.

  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test pretty much everything
"""


import os
import sys
import thread

from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
import rospy

from csa_msgs.msg import CSADirective, CSAResponse


class ReflexiveModule(object):
    """
    RSE reflexive_layer module for a dual arm robot.
    
    TODO: Test pretty much everything.
    """
    
    def __init__(self):
        """
        Module initialization
        """
        
        # Get CWD
        home_cwd = os.getcwd()
        
        # Initialize rospy node
        rospy.init_node("reflexive_module")
        rospy.loginfo("Node initialized")
        
        # Initialize cleanup for the node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Command line inputs
        #TODO
        
        # Retrieve parameters 
        self.rate = rospy.get_param("~rate", 50.0)
        self.side = rospy.get_param("~side", "")
        self.robot = rospy.get_param("~robot", "")
        
        # Parse out joint information
        self.joints = []
        for name in rospy.get_param("~joints", dict().keys()):
            self.joints.append(name)
        
        # Setup variables
        self.directives = []
        self.status = ""
        self.description = ""
        self.cur_directive = 0
        self.trajectory = JointTrajectory()
        self.commands = []
        
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
        
        # Shutdown node
        rospy.sleep(1)
        rospy.loginfo("Shutting down reflexive_module node")
        
    #==== COMMUNICATIONS ==============================================#
        
    def init_comms(self):
        """
        Initialize the communication interfaces for the module. May use
        ROS patterns or ROSbridge library.
        """
        
        # Message topic names
        up_response_name = "/{}/{}_arm/RL_response".format(self.robot,
                                                       self.side)
        up_directive_name = "/{}/{}_arm/HL_directive".format(self.robot, 
                                                         self.side)
        command_name = "/{}/{}_arm/joint_commands".format(self.robot, 
                                                         self.side)
        state_info = "/{}/state_info".format(self.robot)
        
        # Setup ROS Publishers
        self.response_pub = rospy.Publisher(up_response_name, CSAResponse,
                                            queue_size=1)
        self.command_pub = rospy.Publisher(command_name, JointState,
                                           queue_size=1)
        
        # Setup ROS Subscribers
        #self.state_info_sub = rospy.Subscriber(state_info, $STATETOPIC$, $STATECALLBACK$)
        self.directive_sub = rospy.Subscriber(up_directive_name, CSADirective,
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
        
        return msg
        
    def build_command(self, commands):
        """
        Build a command message (uses JointState message).
        """
        
        # Create joint state message
        msg = JointState()
        
        # Fill out message
        msg.header.stamp = rospy.Time.now()
        msg.position = commands
        
        return msg
        
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
        module_node = ReflexiveModule()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
