#!/usr/bin/env python


"""
  Reflexive layer module for both robotic arms in a dual arm setup.

  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test pretty much everything
  TODO: Improve documentation
"""


import copy
import os
import serial
import sys
import thread
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rse_dam_msgs.msg import HLtoRL, RLtoHL, DualArmState
from tactics.relative_jacobian import RelativeJacobianController

# Retrieve modules from elsewhere in the catkin workspace
file_dir = sys.path[0]
sys.path.append(file_dir + "/..")
sys.path.append(file_dir + "/../..")
from common import params
from csa_msgs.msg import TimedPoseArray


class UnifiedReflexiveModule(object):
    """
    RSE reflexive layer for a dual arm robot for unified control of both
    arms from one reflexive layer.
    
    TODO: Test pretty much everything.
    """
    
    def __init__(self):
        """
        Module initialization
        """
        
        # Get CWD
        home_cwd = os.getcwd()
        
        # Get command line arguments
        param_files = sys.argv[1]
        param_dir = sys.argv[2]
        
        # Initialize rospy node
        self.node_name = "reflexive_module"
        rospy.init_node(self.node_name)
        rospy.loginfo("'{}' node initialized".format(self.node_name))
        
        # Get main parameters from file
        self.global_frame = "world"
        param_dict = params.retrieve_params_yaml_files(param_files, param_dir)
        self.rate = 20.0#param_dict["rate"]
        self.robot = param_dict["robot"]
        self.connections = param_dict["connections"]
        self.local_ip = self.connections["left_arm"]
        ref_frame = param_dict["reference_frame"]
        joints = param_dict["joints"]
        
        # Setup master arm
        self.master = "left" #<-- TODO: improve this
        
        # Get full joint names
        self.arm_joints = joints["arm"]
        self.left_arm_joints = ["left_{}_joint".format(joint) for joint in joints["arm"]]
        self.left_eef_joints = ["left_{}_joint".format(joint) for joint in joints["eef"]]
        self.right_arm_joints = ["right_{}_joint".format(joint) for joint in joints["arm"]]
        self.right_eef_joints = ["right_{}_joint".format(joint) for joint in joints["eef"]]
        
        # State variables
        self.arm_state = [None]*7
        
        # Control parameters
        self.poses = TimedPoseArray()
        self.trajectory = JointTrajectory()
        self.new_command = False
        self.state = "start"
        self.status = 0
        
        # Initialize cleanup for the node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Initialize communications
        self.init_comms()
        rospy.loginfo("Communcation interfaces setup")
        
        # Run the module
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
        rospy.loginfo("Shutting down '{}' node".format(self.node_name))
        
    #==== COMMUNICATIONS ==============================================#
        
    def init_comms(self):
        """
        Initialize the communication interfaces for the module. May use
        ROS patterns or ROSbridge library.
        """
        
        # Message topic names
        RLtoHL_name = "{}/RLtoHL".format(self.robot)
        HLtoRL_name = "{}/HLtoRL".format(self.robot)
        state_name = "{}/state_estimate".format(self.robot)
        left_command_name = "{}/left_arm/joint_commands".format(self.robot)
        right_command_name = "{}/right_arm/joint_commands".format(self.robot)
        
        # Setup ROS Publishers
        self.RLtoHL_pub = rospy.Publisher(RLtoHL_name,
                                          RLtoHL,
                                          queue_size=1)
        self.left_command_pub = rospy.Publisher(left_command_name,
                                                JointState,
                                                queue_size=1)
        self.right_command_pub = rospy.Publisher(right_command_name,
                                                 JointState,
                                                 queue_size=1)
        
        # Setup ROS Subscribers
        self.HLtoRL_sub = rospy.Subscriber(HLtoRL_name,
                                           HLtoRL,
                                           self.HLtoRL_cb)
        self.state_sub = rospy.Subscriber(state_name,
                                          DualArmState,
                                          self.state_cb)
                                          
    def build_RLtoHL_msg(self):
        """
        Build a message to respond to the habitual layer.
        """
        
        msg = RLtoHL()
        msg.status = self.status
        
        return msg
        
    def build_command_msg(self, command, side):
        """
        Build a joint command message to send to the robot interface.
        
        TODO: make input more general to work with wider variety of
        controllers/hardware.
        """
        
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        if command != None:
            command.append(0.0) # <-- TODO: improve this
        msg.position = command
        
        # Handle joint names
        if side == "left":
            msg.name = self.left_arm_joints + self.left_eef_joints
        elif side == "right":
            msg.name = self.right_arm_joints + self.right_eef_joints
        
        return msg
                                          
    def HLtoRL_cb(self, msg):
        """
        Callback function for command sent from the habitual layer.
        """
        
        # Acquire lock
        self.lock.acquire()
        
        try:
            
            # Extract control info
            self.new_command = bool(msg.new_command)
            self.control_type = msg.ctrl_type
            self.poses = msg.poses
        
        finally:
            
            # Release lock
            self.lock.release()
        
    def state_cb(self, msg):
        """
        Callback function for arm state from the state estimator.
        """
        
        # Acquire lock
        self.lock.acquire()
        
        try:
            
            # Extract state info
            if self.master == "left":
                self.master_joint_state = msg.left_joint_state
                self.master_jacobian = self.convert_J_to_numpy_array(msg.left_jacobian)
                self.master_eef_state = msg.left_eef_state
                self.slave_joint_state = msg.right_joint_state
                self.slave_jacobian = self.convert_J_to_numpy_array(msg.right_jacobian)
                self.slave_eef_state = msg.right_eef_state
            elif self.master == "right":
                self.master_joint_state = msg.right_joint_state
                self.master_jacobian = self.convert_J_to_numpy_array(msg.right_jacobian)
                self.master_eef_state = msg.right_eef_state
                self.slave_joint_state = self.convert_J_to_numpy_array(msg.left_joint_state)
                self.slave_jacobian = msg.left_jacobian
                self.slave_eef_state = msg.left_eef_state
            self.rel_jacobian = self.convert_J_to_numpy_array(msg.rel_jacobian)
            
            # 'Package' state information
            self.arm_state = [self.master_joint_state,
                              self.master_jacobian,
                              self.master_eef_state,
                              self.slave_joint_state,
                              self.slave_jacobian,
                              self.slave_eef_state,
                              self.rel_jacobian]
            
        finally:
            
            # Release lock
            self.lock.release()
            
    def convert_J_to_numpy_array(self, J_msg):
        """
        Convert the individual rows of the Jacobian found in the 'Jacobian'
        message type into a numpy array representaion of the Jacobian.
        """
        
        # Get out each row of the Jacobian
        row0 = J_msg.row0
        row1 = J_msg.row1
        row2 = J_msg.row2
        row3 = J_msg.row3
        row4 = J_msg.row4
        row5 = J_msg.row5
        
        # Put into numpy array
        J = np.array([row0, row1, row2, row3, row4, row5])
        
        return J
    
    #==== STATE MACHINE ===============================================#
        
    def state_machine(self):
        """
        Perform one update cycle of the module state machine.
        """
        
        # Wait for new command from HL
        if self.state == "start":
            self.state = "standby"
        elif self.state == "standby":
            new_command = self.wait_for_command()
            if new_command:
                self.state = "setup_controller"
        
        # give the controller with informaton on task
        elif self.state == "setup_controller":
            self.controller = self.setup_controller()
            self.state = "execute"
        
        # Execute the trajectory
        elif self.state == "execute":
            goal_a, goal_b, status = self.controller.update(self.arm_state)
            if status == 0:
                pass
            elif status == 1:
                self.status = 1
                if goal_a != None and goal_b!= None:
                    command_left, command_right = self.orient_commands(goal_a, goal_b)
                    self.left_command_pub.publish(command_left)
                    self.right_command_pub.publish(command_right)
            elif status == 2:
                self.status = 2
                self.state = "reset"
            
            rltohl_msg = self.build_RLtoHL_msg()
            self.RLtoHL_pub.publish(rltohl_msg)
            
        # Reset module
        elif self.state == "reset":
            self.controller = None
            self.new_command = False
            self.status = 0
            self.state = "standby"
        
    #==== MODULE BEHAVIORS ============================================#
            
    def setup_controller(self):
        """
        Determine which kind of controller to initilize and feed task 
        setup information.
        """
            
        # Setup relative Jacobian controller
        controller = RelativeJacobianController(self.arm_joints, 
                                                self.rate)
        controller.input_new_trajectory(self.poses)
            
        return controller
        
    def wait_for_command(self):
        """
        Wait for the HL to send down a new goal.
        """
        
        if self.new_command == True:
            return True
        else:
            return False
            
    def orient_commands(self, goal_a, goal_b):
        """
        Based on which arm is the master, figure out which arm to send
        each of the commands to.
        """
        
        if self.master == "left":
            command_left = self.build_command_msg(goal_a, "left")
            command_right = self.build_command_msg(goal_b, "right")
        elif self.master == "right":
            command_left = self.build_command_msg(goal_b, "left")
            command_right = self.build_command_msg(goal_a, "right")
            
        return command_left, command_right


if __name__ == "__main__":
    try:
        UnifiedReflexiveModule()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
