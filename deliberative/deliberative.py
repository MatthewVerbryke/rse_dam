#!/usr/bin/env python

"""
  Main deliberative-layer module for one robotic arm in a dual-arm setup.

  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test pretty much everything
"""


import math
import os
import sys
import thread

from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import RobotTrajectory
import rospy

from csa_msgs.msg import CSADirective, CSAResponse


class DeliberativeModule(object):
    """
    The $MODULE_NAME$ object class
    """
    
    #==== INITIALIZATION ==============================================#
    
    def __init__(self):
        """
        Module initialization
        """
        
        # Get CWD
        home_cwd = os.getcwd()

        # Initialize rospy node
        rospy.init_node("deliberative_module")
        rospy.loginfo("Node initialized")
        
        # Initialize cleanup for the node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Command line inputs
        #TODO
        
        # Setup parameters 
        self.rate = rospy.get_param("~rate", 100.0)
        self.robot = rospy.get_param("~robot", "")
        self.connection = rospy.get_param("secondary_ip", "")
        self.ref_frame = rospy.get_param("~reference_frame", "world")
        
        # Parse out joint information
        self.joints = []
        sefl.joints_max_velocity = []
        for name in rospy.get_param("~joints", dict().keys()):
            self.joints.append(name)
            max_velocity = rospy.get_param("~joints/"+name+"/max_speed")
            self.joints_max_velocity.append(math.radians(max_velocity)
        
        # Setup variables
        self.directives = []
        self.status = ""
        self.description = ""
        self.seq_count = 0
        self.object_position = Pose()
        self.object_goal = Pose()
        self.grasps = [Pose(), Pose()]
        self.offsets = [0.0, 0.0]
        self.move_speed = 0.0
        self.old_goal = Pose()
        self.trajectories = [RobotTrajectory(), RobotTrajectory()]
        self.left_response = CSAResponse()
        self.right_response = CSAResponse()
        self.new_directive_flag = False
        self.left_directive_recieved_flag = False
        self.right_directive_recieved_flag = False
        
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
        rospy.loginfo("Shutting down 'deliberative_module' node")
        
    #==== COMMUNICATIONS ==============================================#
        
    def init_comms(self):
        """
        Initialize the communication interfaces for the module. May use
        ROS patterns or ROSbridge library.
        """
        
        # Message topic names
        up_directive = "{}/OP_directive".format(self.robot)
        up_response = "{}/DL_response".format(self.robot)
        left_directive = "{}/left_arm/DL_directive".format(self.robot)
        right_directive = "{}/right_arm/DL_directive".format(self.robot)
        left_response = "{}/left_arm/HL_response".format(self.robot)
        right_response = "{}/right_arm/HL_response".format(self.robot)
        state_info = "{}/state_info".format(self.robot)
        
        # Setup ROS Publishers
        self.up_response_pub = rospy.Publisher(up_response, CSAResponse,
                                               queue_size=1)
        self.left_directive_pub = rospy.Publisher(left_directive,
                                                  CSADirective,
                                                  queue_size=1)
        self.right_directive_pub = rospy.Publisher(right_directive,
                                                   CSADirective,
                                                   queue_size=1)
        
        # Setup ROS Subscribers
        #self.state_info_sub = rospy.Subscriber(state_info, $STATETOPIC$, $STATECALLBACK$)
        self.up_directive_sub = rospy.Subscriber(up_directive, CSADirective,
                                                 self.directive_cb)
        self.left_response_sub = rospy.Subscriber(left_response,
                                                  CSAResponse,
                                                  self.left_response_cb)
        self.right_response_sub = rospy.Subscriber(right_response,
                                                   CSAResponse,
                                                   self.right_response_cb)
        
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
            
    def left_response_cb(self, msg):
        """
        Callback function for responses sent from the left habitual
        module.
        """
        
        # Lock
        self.lock.acquire()
        
        # Retreive Message
        self.left_response = msg
        if self.msg.msg_recieved = True
            self.left_directive_recieved_flag = True
            
        # Release
        self.lock.release()
            
    def right_response_cb(self, msg):
        """
        Callback function for responses sent from the right habitual
        module.
        """
        
        # Lock
        self.lock.acquire()
        
        # Retreive Message
        self.right_response = msg
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
        module_node = DeliberativeModule()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
