#!/usr/bin/env python


"""
  Reflexive layer module for one robotic arm in a dual arm setup.

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
from tactics.follow_controller import FollowController
from tactics.relative_jacobian import RelativeJacobianController

# Retrieve modules from elsewhere in the catkin workspace
file_dir = sys.path[0]
sys.path.append(file_dir + "/..")
sys.path.append(file_dir + "/../..")
from common import params
from csa_msgs.msg import TimedPoseArray


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
        
        # Get command line arguments
        param_files = sys.argv[1]
        param_dir = sys.argv[2]
        self.side = sys.argv[3]
        
        # Initialize rospy node
        self.node_name = "{}_reflexive_module".format(self.side)
        rospy.init_node(self.node_name)
        rospy.loginfo("'{}' node initialized".format(self.node_name))
        
        # Get main parameters from file
        self.global_frame = "world"
        param_dict = params.retrieve_params_yaml_files(param_files, param_dir)
        self.rate = 30#param_dict["rate"]
        self.robot = param_dict["robot"]
        self.connections = param_dict["connections"]
        self.local_ip = self.connections["{}_arm".format(self.side)]
        ref_frame = param_dict["reference_frame"]
        joints = param_dict["joints"]
        
        # Get full joint names
        self.arm_joints = ["{}_{}_joint".format(self.side, joint) for joint in joints["arm"]]
        self.eef_joints = ["{}_{}_joint".format(self.side, joint) for joint in joints["eef"]]
        
        # State variables
        self.arm_state = JointState()
        
        # Control parameters
        self.control_type = None
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
        RLtoHL_name = "RLtoHL"
        HLtoRL_name = "HLtoRL"
        joint_state_name = "{}/{}_arm/joint_states".format(self.robot,
                                                          self.side)
        command_name = "{}/{}_arm/commands".format(self.robot,
                                                  self.side)
        
        # Setup ROS Publishers
        self.RLtoHL_pub = rospy.Publisher(RLtoHL_name,
                                          RLtoHL,
                                          queue_size=1)
        self.command_pub = rospy.Publisher(command_name,
                                           JointState,
                                           queue_size=1)
        
        # Setup ROS Subscribers
        self.HLtoRL_sub = rospy.Subscriber(HLtoRL_name,
                                           HLtoRL,
                                           self.HLtoRL_cb)
        self.state_sub = rospy.Subscriber(joint_state_name,
                                          JointState,
                                          self.state_cb)
                                          
    def build_RLtoHL_msg(self):
        """
        Build a message to respond to the habitual layer.
        """
        
        msg = RLtoHL()
        msg.status = self.status
        
        return msg
        
    def build_command_msg(self, command):
        """
        Build a joint command message to send to the robot interface.
        
        TODO: make input more general to work with wider variety of
        controllers/hardware.
        """
        
                
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.arm_joints + self.eef_joints
        if command != None:
            command.append(0.0) # <-- TODO: improve this
        msg.position = command
        
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
            self.ctrl_type = msg.ctrl_type
            self.poses = msg.poses
            self.trajectory = msg.trajectory
        
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
            
            # Store state message
            self.arm_state = msg
        
        finally:
            
            # Release lock
            self.lock.release()       
    
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
            goal, status = self.controller.update(self.arm_state)
            if status == 0:
                pass
            elif status == 1:
                self.status = 1
                if goal != None:
                    command = self.build_command_msg(goal)
                    self.command_pub.publish(command)
            elif status == 2:
                self.status = 2
                self.state = "reset"
                self.hold = 0
            
        # Reset module
        elif self.state == "reset":
            if self.hold < 6:
                self.hold += 1
                self.status = 2
            else:
                self.status = 2
                self.new_command = False
                self.state = "standby"
        
        # Publish a return message to the reflexive layer at all times
        rltohl_msg = self.build_RLtoHL_msg()
        self.RLtoHL_pub.publish(rltohl_msg)
            
    #==== MODULE BEHAVIORS ============================================#
            
    def setup_controller(self):
        """
        Determine which kind of controller to initilize and feed task 
        setup information.
        """
        
        # Setup follow controller
        if self.ctrl_type == 1:
            joint_traj = self.trajectory.joint_trajectory
            controller = FollowController(self.arm_joints, 
                                          self.rate)
            controller.input_new_trajectory(joint_traj)
        else:
            rospy.logerr("Desired controller type not recognized")
            return None
            
        return controller
        
    def wait_for_command(self):
        """
        Wait for the HL to send down a new goal.
        """
        
        if self.new_command == True:
            return True
        else:
            return False
        
        
if __name__ == "__main__":
    try:
        ReflexiveModule()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
