#!/usr/bin/env python

"""
  State estimation module for one robotic arm in a dual-arm setup.
  
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

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Pose
import numpy as np
from sensor_msgs.msg import JointState
import rospy

from csa_msgs.msg import CSADirective, CSAResponse
from rse_dam_msgs.msg import DualArmState
from tactics.jacobian import DualArmJacobianSolver
from tactics.eef_state import EndEffectorStateSolver


class StateEstimationModule(object):
    """
    RSE state estimation module for a dual-arm robot.
    """
    
    #==== INITIALIZATION ==============================================#
    
    def __init__(self):
        """
        Module initialization
        
        TODO: Test
        """
        
        # Get CWD
        home_cwd = os.getcwd()
        
        # Initialize rospy node
        rospy.init_node("state_estimation")
        rospy.loginfo("Node initialized")
        
        # Initialize cleanup for the node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Setup parameters 
        self.rate = rospy.get_param("~rate", 50.0)
        self.robot = rospy.get_param("~robot", "")
        self.connection = rospy.get_param("~secondary_ip", "")
        self.ref_frame = rospy.get_param("~reference_frame", "world")
        
        # Parse out joint information (currently assume both arms are same)
        self.joints = []
        for name in rospy.get_param("~joints", dict().keys()):
            self.joints.append(name)
        self.num_joints = len(self.joints)
        
        # Setup variables
        self.seq_in = 0
        self.ctrl_seq = 0
        self.directives = []
        self.ctrl_directive = None
        self.arb_status = 0
        self.ctrl_status = 0
        self.am_status = 0
        self.new_directive = True
        self.executing = False
        
        # Setup state storage variables
        self.left_joint_state = JointState()
        self.left_jacobian = np.zeros(6,self.num_joints)
        self.left_eef_state = Pose()
        self.right_joint_state = JointState()
        self.right_jacobian = np.zeros(6,self.num_joints)
        self.right_eef_state = Pose()
        self.relative_jacobian = np.zeros(6,self.num_joints)
        self.target_state = Pose()
        self.left_actuator_health = DiagnosticArray()
        self.right_actuator_health = DiagnosticArray()
        
        # Initialize communications
        self.init_comms()
        rospy.loginfo("Communcation interfaces setup")
        
        # Start state machine
        self.main()
    
    def cleanup(self):
        """
        Function to cleanly stop module on shutdown command.
        """
        rospy.sleep(1)
        rospy.loginfo("Shutting down state_estimation module")
        
    #==== COMMUNICATIONS ==============================================#
        
    def init_comms(self):
        """
        Initialize the communication interfaces for the module. May use
        ROS patterns or ROSbridge library.
        
        TODO: Test
        """
        
        # Message topic names
        robot_state = "{}/state_est".format(self.robot)
        response = "{}/estimator/response".format(self.robot)
        left_joint_state = "{}/left_arm/joint_states".format(self.robot)
        tf = "{}/left_arm/tf".format(self.robot)
        right_joint_state = "{}/right_arm/joint_states".format(self.robot)
        directive = "{}/estimator/directive".format(self.robot)
        
        # Setup ROS Publishers
        self.robot_state_pub = rospy.Publisher(robot_state, DualArmState,
                                               queue_size=1)
        self.response_pub = rospy.Publisher(response, CSAResponse,
                                            queue_size=1)
        
        # Setup ROS Subscribers
        self.left_joint_sub = rospy.Subscriber(left_joint_state,
                                               JointState,
                                               self.left_joint_state_cb)
        self.right_joint_sub = rospy.Subscriber(right_joint_state,
                                                JointState,
                                                self.right_joint_state_cb)
        self.directive_sub = rospy.Subscriber(directive, CSADirective,
                                              self.directive_cb)
        
    def build_state_message(self, ctrl_out):
        """
        Build a dual arm robot state message
        
        TODO: Test
        """
        
        # Create state message
        msg = DualArmState()
        
        # Fill out message
        msg.left_joint_state = self.right_joint_state
        msg.left_jacobian = ctrl_out[0]
        msg.left_eef_state = ctrl_out[3]
        msg.left_status = self.left_actuator_health
        msg.right_joint_state = self.right_joint_state
        msg.right_jacobian = ctrl_out[1]
        msg.right_eef_state = ctrl_out[4]
        msg.right_status = self.right_actuator_health
        msg.rel_jacobian = ctrl_out[2]
        msg.target_state = self.target_state
        
        return msg
        
    def build_response(self, poses, joint_trajectory):
        """
        Build a CSA response message
        
        TODO: Test
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
        
    def left_joint_state_cb(self, msg):
        """
        Callback function for left arm joint state messages
        """
        self.left_joint_state = msg
        
    def right_joint_state_cb(self, msg):
        """
        Callback function for right arm joint state messages
        """
        self.right_joint_state = msg

    def directive_cb(self, msg):
        """
        Callback function for directives sent from the deliberative
        module.
        
        TODO: Test
        """
        
        # Only store directive if it has new sequence number
        new_seq = msg.header.seq
        if new_seq == self.seq_in:
            pass
        elif new_seq < self.seq_in:
            rospy.logerr("new directive sequence number is less than internal count")
        else:
            self.new_directive = True
            self.seq_in += 1
            self.up_msg = msg
    
    #==== STATE MACHINE ===============================================#
        
    def main(self):
        """
        Main execution loop for the module.
        
        TODO: Test
        """
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.lock.acquire()
            self.run_components()
            self.lock.release()
            r.sleep()
    
    def run_components(self):
        """
        Perform one update cycle of the each module component.
        
        TODO: Test
        """
        
        # Get a lock
        self.lock.acquire()
        
        try:
            
            # Run each of the CSA components
            self.arb_to_ctrl = self.arbitration(self.ctrl_status)
            self.ctrl_to_am, self.ctrl_to_tact = self.control(self.arb_to_ctrl,
                                                              self.tact_to_ctrl)
            self.tact_to_ctrl = self.tactics(self.ctrl_to_tact)
            self.am_to_ctrl = self.activity_manager(self.ctrl_to_am)
        
        # Release the lock
        self.lock.release()
        
    #==== CSA SUBMODULES ==============================================#
        
    def arbitration(self, ctrl_status):
        """
        Manages the overall behavior of the estimator: 
            - Comuputes a "merged directive" from all directives recieved 
              from commanding module(s)
            - Issues merged directive to send to the control component
            - Reports status (accept, reject, error,...) to the
              commanding module(s)
        
        TODO: Test
        """
        
        # Blank response
        merged_directive = None
        
        # Standby
        if self.arb_status == 0:
            if new_directive:
                self.directives.append(self.up_msg)
                self.arb_status = 1
            else:
                pass
        
        # Merging directives
        elif self.arb_status == 1:
            if len(self.directives) == 0:
                self.arb_status = 0
            else:
                merge_result = priority_merge(self.directives)# <----- TODO: FILL IN FUNCTION NAME
                merged_directive = merge_result[0]
                self.directives = merge_result[1]
                if merged_directive != None:
                    self.arb_status = 2
                else:
                    self.arb_status = 4
            self.new_directive = False
        
        # Wait for execution of merged directive
        elif self.arb_status == 2:
            if ctrl_status = 1 or ctrl_status = 2:
                pass
            elif ctrl_status = 3:
                self.arb_status = 3
            elif ctrl_status = 4:
                self.arb_status = 4                
        
        # Report successful execution of directive
        elif self.arb_status == 3:
            response_msg = self.build_response()
            for i in range(0,5):
                response_pub.publish(response_msg)
            self.arb_status == 1
        
        # Report a failure in executing the directive
        elif self.arb_status == 4:
            response_msg = self.build_response()
            for i in range(0,5):
                response_pub.publish(response_msg)
            self.arb_status == 1
            
        return merged_directive
        
    def control(self, arb_directive, tactic):
        """
        Performs overall function of the estimator:
            - Estimates the state of the system using sensor input
            - Reports failure to the arbitration component
            
        TODO: Test
        """
        
        # Blank returns
        ctrl_out = None
        tact_request = None
        
        # Standby
        if self.ctrl_status == 0:
            if arb_directive == None:
                pass
            else:
                self.ctrl_status = 1
                self.ctrl_directive = arb_directive
        
        # Poll tactics
        elif self.ctrl_status == 1:
            if self.tactic == None and tactic == None:
                tact_request = self.ctrl_directive
            elif self.tactic == None and tactic != None:
                self.tactic = tactic
                self.ctrl_status = 2
        
        # Execution
        elif self.ctrl_status == 2:
            ctrl_out = []
            for tact in self.tactic
                result, status = tact.update()
                if status == "fail":
                    self.ctrl_status = 4
                elif status == "ok":
                    ctrl_out.extend(result)
            self.parse_results(ctrl_out)
        
        # Success in directive execution
        elif self.ctrl_status == 3:
            self.ctrl_status = 0
            # TODO: more complex handeling of completion process?
        
        # Failure handling
        elif self.ctrl_status == 4:
            self.ctrl_status = 0
            # TODO: failure handling/plan repair?
            
        return ctrl_out, tact_request
        
    def tactics(self, tactic_request):
        """
        Generates estimation tactic(s) for the estimator to use based on
        the merged directive.
        
        TODO: Test
        """
        
        if tactic_request == "default":
            jacobian_solver = DualArmJacobianSolver(self.robot,
                                                    self.left_base,
                                                    self.left_eef,
                                                    self.right_base,
                                                    self.right_eef)
            eef_state_solver = EndEffectorStateSolver(self.robot,
                                                      self.base_frame,
                                                      self.left_eef,
                                                      self.right_eef)
            tactic = [jacobian_solver, eef_state_solver]
            
        return tactic
        
    def activity_manager(self, ctrl_out):
        """
        Handles the output of state information from the estimator.
        Simply publishes state to other modules without response.
        
        TODO: Test
        """
        
        # Build a dual arm state message
        msg = self.build_state_message(ctrl_out)
        
        # Publish state information
        self.robot_state_pub.publish(msg)
        
    #==== OTHER FUNCTIONS =============================================#
    
    def parse_results(self, results):
        """
        Place the results of tactics update in the correct state
        variables.
        
        TODO: Test
        """
        
        self.left_jacobian = results[0]
        self.right_jacobian = results[1]
        self.relative_jacobian = results[2]
        self.left_eef_state = results[3]
        self.right_eef_state = results[4]


if __name__ == "__main__":
    try:
        module_node = StateEstimationModule()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
