#!/usr/bin/env python

"""
  Main habitual-layer module for one robotic arm in a dual-arm setup.

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

import rospy

from csa_msgs.msg import CSADirective, CSAResponse


class HabitualModule(object):
    """
    The HabitualModule object class
    """
    
    #==== INITIALIZATION ==============================================#
    
    def __init__(self):
        """
        Module initialization
        """
        
        # Get CWD
        home_cwd = os.getcwd()
        
        # Initialize rospy node
        rospy.init_node("habitual_module")
        rospy.loginfo("Node initialized")
        
        # Initialize cleanup for the node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Command line inputs
        #TODO
        
        # Setup parameters 
        self.rate = rospy.get_param("~rate", 100.0)
        self.side = rospy.get_param("~side", "")
        self.robot = rospy.get_param("~robot", "")
        self.connection = rospy.get_param("~primary_ip", "")
        self.ref_frame = rospy.get_param("~reference_frame", "world")
        
        # Parse out joint information
        self.joints = []
        for name in rospy.get_param("~joints", dict().keys()):
            self.joints.append(name)
        
        # Setup variables
        self.directives = []
        self.RL_response = CSAResponse()
        self.status = ""
        self.description = ""
        self.cur_directive = 0
        self.seq = 0
        self.new_directive_flag = False
        self.directive_recieved_flag = False
        
        # Initialize communications
        self.init_comms()
        rospy.loginfo("Communcation interfaces setup")
        
        # Start state machine
        self.run_state_machine()

    def run_state_machine(self):
        """
        Main execution loop for the module.
        """
        
        while not rospy.is_shutdown():
            self.state_machine()
            rospy.sleep(0.01)
    
    def cleanup(self):
        """
        Function to cleanly stop module on shutdown command.
        """
        
        rospy.sleep(1)
        rospy.loginfo("Shutting down habitual_module node")
        
    #==== COMMUNICATIONS ==============================================#
        
    def init_comms(self):
        """
        Initialize the communication interfaces for the module. May use
        ROS patterns or ROSbridge library.
        """
        
        # Message topic names
        up_directive = "{}/{}_arm/DL_directive".format(self.robot, 
                                                         self.side)
        up_response = "{}/{}_arm/HL_response".format(self.robot,
                                                       self.side)
        down_directive = "{}/{}_arm/HL_directive".format(self.robot, 
                                                         self.side)
        down_response = "{}/{}_arm/RL_response".format(self.robot,
                                                            self.side)
        state_info = "{}/state_info".format(self.robot)
        
        # Setup ROS Publishers
        self.up_response_pub = rospy.Publisher(up_response, CSAResponse,
                                               queue_size=1)
        self.down_directive_pub = rospy.Publisher(down_directive, CSADirective,
                                                  queue_size=1)
        
        # Setup ROS Subscribers
        #self.state_info_sub = rospy.Subscriber(state_info, $STATETOPIC$, $STATECALLBACK$)
        self.up_directive_sub = rospy.Subscriber(up_directive, CSADirective,
                                              self.directive_cb)
        self.down_response_sub = rospy.Subscriber(down_response, CSAResponse,
                                                  self.response_cb)
        
    def build_response(self, poses, joint_trajectory):
        """
        Build a CSA response message
        """
        
        # Create the response message
        msg = CSAResponse()
        
        # Fill out message
        msg.header.seq = self.cur_directive
        msg.header.stamp = rospy.Time.now()
        msg.msg_received = self.new_msg
        msg.status = self.status
        msg.response = self.description
        msg.response_poses = poses
        msg.response_joint_trajectory = joint_trajectory
        
        return msg
        
    def build_directive(self, action, target, start_time, poses, joint_trajectory):
        """
        Build a CSA directive message
        """
        
        # Create the directive message
        msg = CSADirective()
        
        # Fill out message
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        msg.action = action
        msg.start_time = start_time
        msg.target = target
        msg.target_poses = poses
        msg.joint_trajectory = joint_trajectory
        
        return msg
        
    def directive_cb(self, msg):
        """
        Callback function for directives sent from the deliberative
        module.
        """
        
        # Lock
        self.lock.acquire()
        
        # Only store directive if it has new sequence number
        new_seq = msg.header.seq
        if new_seq == self.seq_count:
            pass
        else:
            self.directives.append(msg)
            self.new_directive_flag = True
            
        # Release
        self.lock.release()
            
    def response_cb(self, msg):
        """
        Callback function for responses sent from the reflexive module.
        """
        
        # Lock
        self.lock.acquire()
        
        self.RL_response = msg
        if self.msg.msg_recieved = True
            self.directive_recieved_flag = True
    
        # Release
        self.lock.release()
    
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
        module_node = HabitualModule()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
