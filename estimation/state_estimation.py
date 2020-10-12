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


import ast
import os
import sys
import thread

from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
from geometry_msgs.msg import Pose
import numpy as np
from sensor_msgs.msg import JointState
import rospy
import tf

from rse_dam_msgs.msg import DualArmState
from tactics.jacobian import DualArmJacobianSolver
from tactics.eef_state import EndEffectorStateSolver

# Retrieve modules from elsewhere in the catkin workspace
file_dir = sys.path[0]
sys.path.append(file_dir + "/..")
sys.path.append(file_dir + "/../..")
from common import params
from communication.rse_packing import pack_dual_arm_state
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


class PsuedoStateEstimationModule(object):
    """
    The 'PsuedoStateEstimatorModule' object class
    """
    
    #==== INITIALIZATION ==============================================#
    
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
        rospy.init_node("state_estimation_module")
        rospy.loginfo("'state_estimation_module' node initialized")
        
        # Get main parameters from file
        param_dict = params.retrieve_params_yaml_files(param_files, param_dir)
        self.rate = param_dict["rate"]
        self.robot = param_dict["robot"]
        joints = param_dict["joints"]
        self.connections = param_dict["connections"]
        self.local_ip = self.connections["main"]
        self.global_frame = "world"
        ref_frame = param_dict["reference_frame"]
        self.num_joints = len(joints["arm"])
        left_base = "left_" + param_dict["base_link"]
        right_base = "right_" + param_dict["base_link"]
        left_eef = "left_" + param_dict["eef_link"]
        right_eef = "right_" + param_dict["eef_link"]
        
        # Get arm joint names
        left_name = []
        right_name = []
        for i in range(0,self.num_joints):
            left_name.append("left_" + joints["arm"][i] + "_joint")
            right_name.append("right_" + joints["arm"][i] + "_joint")
            
        # Store arm information TODO: handle end-effector joints
        index = [0]*self.num_joints
        self.arms = {"left_arm": {"name": left_name,
                                  "index": index,
                                  "base": left_base,
                                  "eef": left_eef},
                     "right_arm": {"name": right_name,
                                   "index": index,
                                   "base": right_base,
                                   "eef": right_eef},}
                                   
        # Set master arm (TODO: make this a variable)
        self.master = "left"
        
        # Initialize the state storage variable        
        self.state = DualArmState()
        self.state.left_jacobian = np.zeros((6*self.num_joints))
        self.state.right_jacobian = np.zeros((6*self.num_joints))
        self.state.rel_jacobian = np.zeros((6*self.num_joints))
        
        # Initialize state solvers
        self.jacobian_solver = DualArmJacobianSolver(self.robot,
                                                     self.arms,
                                                     self.master)
        self.eef_solver = EndEffectorStateSolver(self.robot,
                                                 self.arms)
        
        # Initialize cleanup for the node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Initialize communications
        self.init_comms()
        rospy.loginfo("Communcation interfaces setup")
        
        # Wait for source topics to start publishing
        rospy.sleep(2.5)
        
        # Start state machine
        self.run_state_machine()

    def run_state_machine(self):
        """
        Main execution loop for the module.
        """
        
        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.state_machine()
            r.sleep()
    
    def cleanup(self):
        """
        Function to cleanly stop module on shutdown command.
        """
        
        rospy.sleep(1)
        rospy.loginfo("Shutting down 'state_estimation_module' node")
        
    #==== COMMUNICATIONS ==============================================#
        
    def init_comms(self):
        """
        Initialize the communication interfaces for the module. May use
        ROS patterns or ROSbridge library.
        
        TODO: Test
        """
        
        # Message topic names
        robot_state = "{}/state_estimate".format(self.robot)
        left_joint_state = "{}/left_arm/joint_states".format(self.robot)
        tf = "{}/left_arm/tf".format(self.robot)
        right_joint_state = "{}/right_arm/joint_states".format(self.robot)
        
        # Setup ROS Subscribers
        self.left_joint_sub = rospy.Subscriber(left_joint_state,
                                               JointState,
                                               self.left_joint_state_cb)
        self.right_joint_sub = rospy.Subscriber(right_joint_state,
                                                JointState,
                                                self.right_joint_state_cb)
                                                
        # Setup local ROS Publishers
        self.local_state_pub = rospy.Publisher(robot_state,
                                               DualArmState,
                                               queue_size=1)
        rospy.loginfo("Local ROS subscribers and publishers initialized")
        
        # Setup ROSbridge Publishers
        self.pubs = []
        for key, ip in self.connections.items():
            if ip == self.local_ip:
                pass
            else:
                new_ws_pub = rC.RosMsg("ws4py",
                                       "ws://"+ip+":9090/", 
                                       "pub",
                                       robot_state,
                                       "rse_dam_msgs/DualArmState",
                                       pack_dual_arm_state)
                self.pubs.append(new_ws_pub)
                
        # Display communication information to terminal
        rospy.loginfo("{} rosbridge publishers initialized".format(len(self.pubs)))
        
    def build_state_message(self, ctrl_out):
        """
        Build a dual arm robot state message
        
        TODO: Test
        """
        
        # Convert jacobians from numpy array to 1-D python lists
        left_J_nest = ctrl_out[0].tolist()
        left_jacobian = [val for sublist in left_J_nest for val in sublist]
        right_J_nest = ctrl_out[1].tolist()
        right_jacobian = [val for sublist in right_J_nest for val in sublist]
        rel_J_nest = ctrl_out[2].tolist()
        rel_jacobian = [val for sublist in rel_J_nest for val in sublist]
        
        # Put the new data into the state variable
        self.state.left_jacobian = left_jacobian
        self.state.left_eef_state = ctrl_out[3]
        self.state.right_jacobian = right_jacobian
        self.state.right_eef_state = ctrl_out[4]
        self.state.rel_jacobian = rel_jacobian
        
        # Send the state variable out as the state message
        msg = self.state
        
        return msg
        
    def left_joint_state_cb(self, msg):
        """
        Callback function for the left-arm joint state message
        
        TODO: Test
        """
        
        # Check the index of each joint in joint state
        for i in range(0,self.num_joints):
            name = self.arms["left_arm"]["name"][i]
            idx = self.arms["left_arm"]["index"][i]
            if msg.name[idx] == name:
                pass
            else:
                self.arms["left_arm"]["index"][i] = msg.name.index(name)
        
        # Write the message to the state varibale
        self.state.left_joint_state = msg

    def right_joint_state_cb(self, msg):
        """
        Callback function for the right-arm joint state message
        
        TODO: Test
        """
        
        # Check the index of each joint in joint state
        for i in range(0,self.num_joints):
            name = self.arms["right_arm"]["name"][i]
            idx = self.arms["right_arm"]["index"][i]
            if msg.name[idx] == name:
                pass
            else:
                self.arms["right_arm"]["index"][i] = msg.name.index(name)
        
        # Write the message to the state varibale
        self.state.right_joint_state = msg
    
    #==== STATE MACHINE ===============================================#
        
    def state_machine(self):
        """
        Perform one update cycle of the module state machine.
        """
        
        # Log that we are starting
        rospy.loginfo("Beginning to publish robot state")
        
        # Update Solvers
        jacobians, j_status = self.jacobian_solver.update(self.state)
        eef_poses, e_status = self.eef_solver.update(self.state)
        output = jacobians + eef_poses
        
        # Store in state variable and publish
        if j_status == "ok" and e_status == "ok":
            msg = self.build_state_message(output)
            self.local_state_pub.publish(msg)
            for pub in self.pubs:
                pub.send(msg)


if __name__ == "__main__":
    try:
        PsuedoStateEstimationModule()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
