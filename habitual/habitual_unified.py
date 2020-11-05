
#!/usr/bin/env python

"""
  Reflexive layer module for both robotic arms in a dual arm setup.
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import copy
import os
import sys
import thread

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotTrajectory
import rospy

from rse_dam_msgs.msg import RLtoHL, HLtoRL, DLtoHL, HLtoDL, DualArmState

# Retrieve modules from elsewhere in the catkin workspace
file_dir = sys.path[0]
sys.path.append(file_dir + "/..")
sys.path.append(file_dir + "/../..")
from common import params
from csa_msgs.msg import TimedPoseArray
import deliberative.tactics.trajectories as traj


class UnifiedHabitualModule(object):
    """
    RSE habitual layer for a dual arm robot for unified control of both
    arms from one reflexive layer.
    
    WARN: Currently a prototype; may or may not come back to this
    depending on several things.
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
        self.node_name = "habitual_module"
        rospy.init_node(self.node_name)
        rospy.loginfo("'{}' node initialized".format(self.node_name))
        
        # 'Set-in-stone' parameters TODO: change these to be changeable
        self.global_frame = "world"
        self.rate = 20.0
        self.master = "left"
        
        # Get main parameters from file
        param_dict = params.retrieve_params_yaml_files(param_files, param_dir)
        self.robot = param_dict["robot"]
        self.connections = param_dict["connections"]
        self.local_ip = self.connections["left_arm"]
        ref_frame = param_dict["reference_frame"]
        joints = param_dict["joints"]
        
        # Get full joint names for both arms
        self.arm_joints = joints["arm"]
        self.left_arm_joints = ["left_{}".format(joint) for joint in joints["arm"]]
        self.left_eef_joints = ["left_{}".format(joint) for joint in joints["eef"]]
        self.right_arm_joints = ["right_{}".format(joint) for joint in joints["arm"]]
        self.right_eef_joints = ["right_{}".format(joint) for joint in joints["eef"]]
        self.eef_name = param_dict["eef_link"]
        self.ref_frame = param_dict["reference_frame"]
        
        # Robot state variables
        self.arm_states = [None]*7
        
        # Module parameters
        self.state = "start"
        self.status = 0
        self.fail_info = ""
        self.increment = 0
        
        # Task setup variables
        self.new_command = False
        self.move_type = 0
        self.command = 0
        self.executed = False
        self.start_left_pose = Pose()
        self.goal_left_pose = Pose()
        self.start_right_pose = Pose()
        self.goal_right_pose = Pose()
        
        # Trajectory information variables
        self.current_left_pose = PoseStamped()
        self.current_right_pose = PoseStamped()
        self.pose_trajectory = TimedPoseArray()
        
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
        Main execution loop for the module
        """
        
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.state_machine()
            r.sleep()
            
    def cleanup(self):
        """
        Function to cleanly stop module on shutdown command
        """
        
        # Shutdown node
        rospy.loginfo("Shutting down '{}' node".format(self.node_name))
        rospy.sleep(1)
    
    #==== COMMUNICATIONS ==============================================#
    
    def init_comms(self):
        """
        Initialize the communication interfaces for the module. May use
        ROS patterns or ROSbridge library.
        """
        
        # Message topic names
        HLtoDL_name = "left_HLtoDL"
        DLtoHL_name = "left_DLtoHL"
        HLtoRL_name = "HLtoRL"
        RLtoHL_name = "RLtoHL"
        state_name = "{}/state_estimate".format(self.robot)
        
        # Setup ROS Publishers
        self.HLtoRL_pub = rospy.Publisher(HLtoRL_name,
                                          HLtoRL,
                                          queue_size=1)
        self.HLtoDL_pub = rospy.Publisher(HLtoDL_name,
                                          HLtoDL,
                                          queue_size=1)
        
        # Setup ROS Subscribers
        self.DLtoHL_sub = rospy.Subscriber(DLtoHL_name,
                                           DLtoHL,
                                           self.DLtoHL_cb)
        self.RLtoHL_sub = rospy.Subscriber(RLtoHL_name,
                                            RLtoHL,
                                            self.RLtoHL_cb)
        self.arm_state_sub = rospy.Subscriber(state_name,
                                              DualArmState,
                                              self.state_cb)
        
        # Setup ROSbridge websockets (TODO)
        
    def build_HLtoRL_msg(self, is_new, ctrl_type, trajectory):
        """
        Build a 'HLtoDL' message
        """
        
        # Fill out the HLtoRL message from input info
        msg = HLtoRL()
        msg.new_command = is_new
        msg.ctrl_type = ctrl_type
        msg.poses = trajectory
        msg.trajectory = RobotTrajectory()
        
        return msg        
        
    def build_HLtoDL_msg(self):
        """
        Build a 'HLtoDL' message
        """
        
        # Fill out the HLtoDL message from input info
        msg = HLtoDL()
        msg.status = self.status
        msg.fail_msg = self.fail_info
        
        return msg
    
    def RLtoHL_cb(self, msg):
        """
        Callback function for messages from the reflexive layer.
        """
        
        # Acquire lock
        self.lock.acquire()
        
        try:
            
            #Extract information from message
            self.rl_status = msg.status
            
        finally:
            
            # Release lock
            self.lock.release()
        
    def DLtoHL_cb(self, msg):
        """
        Callback function for messages from the deliberative layer.
        """
        
        # Acquire lock
        self.lock.acquire()
        
        try:
            
            # Extract information from message
            self.new_command = msg.new_cmd
            self.move_type = msg.move_type
            self.goal_poses = msg.poses
            
        finally:
            
            # Release lock
            self.lock.release()
        
    def state_cb(self, msg):
        """
        Callback function for robot state from the state estimator.
        """
        
        # Acquire lock
        self.lock.acquire()
        
        try:
            
            # Extract state info
            if self.master == "left":
                master_joint_state = msg.left_joint_state
                #master_jacobian = self.convert_J_to_numpy_array(msg.left_jacobian)
                master_eef_state = msg.left_eef_state
                slave_joint_state = msg.right_joint_state
                #slave_jacobian = self.convert_J_to_numpy_array(msg.right_jacobian)
                slave_eef_state = msg.right_eef_state
            elif self.master == "right":
                master_joint_state = msg.right_joint_state
                #master_jacobian = self.convert_J_to_numpy_array(msg.right_jacobian)
                master_eef_state = msg.right_eef_state
                slave_joint_state = self.convert_J_to_numpy_array(msg.left_joint_state)
                #slave_jacobian = msg.left_jacobian
                slave_eef_state = msg.left_eef_state
            #rel_jacobian = self.convert_J_to_numpy_array(msg.rel_jacobian)
            
            # 'Package' state information
            self.arm_states = [master_joint_state,
                              None,#master_jacobian,
                              master_eef_state,
                              slave_joint_state,
                              None,#slave_jacobian,
                              slave_eef_state,
                              None]#rel_jacobian]
        
        finally:
            
            # Release lock
            self.lock.release()
        
    #==== STATE MACHINE ===============================================#
        
    def state_machine(self):
        """
        Perform one update cycle of the module state machine.
        """
        
        # Start
        if self.state == "start":
            self.state = "standby"
            self.status = 0
            
        # Wait for new command from HL
        elif self.state == "standby":
            new_command = self.wait_for_command()
            if new_command:
                self.state = "check_reachability"
                self.status = 1
            else:
                pass
        
        # Check the reachability of the goal
        elif self.state == "check_reachability":
            goal_reachable = True #<--TODO
            if goal_reachable == True:
                self.state = self.determine_action()
            else:
                self.state = "fail_up"
                self.status = 3
        
        # Build a master slave trajectory fo the arms
        elif self.state == "build_master_slave_trajectory":
            trajectory_built = self.build_master_slave_trajectory()
            if trajectory_built:
                self.state = "execute_plan"
                self.status = 4
            else:
                self.state = "fail_up"
                self.status = 3
                
        # Execute the planned trajectory
        elif self.state == "execute_plan":
            if not self.executed:
                success = self.execute_trajectory()
                self.executed = True
                if success:
                    self.status = 6
                else:
                    self.state = "fail_up"
                    self.status = 7
                    
            # Signal completion
            elif self.executed and self.status == 6:
                if self.rl_status == 2:
                    self.standby = "standby"
                    self.status = 0
                    self.reset_module()
                else:
                    pass

        # Signal Failure
        elif self.state == "fail_up":
            pass #TODO
            
        # always pass a new message to the DL
        hltodl_msg = self.build_HLtoDL_msg()
        self.HLtoDL_pub.publish(hltodl_msg)
        
    #==== MODULE BEHAVIORS ============================================#

    def wait_for_command(self):
        """
        Wait for the DL to send down a new goal.
        """
        
        # Check the new command flag to see if this is a new command
        if (self.new_command==0):
            return False
        elif (self.new_command==1):
            return True
    
    def determine_action(self):
        """
        Determine what type of planning state to transition to based on DL
        command.
        """
        
        if self.move_type == 0:
            self.standby = "standby"
            self.status = 0
            self.reset_module()
        elif self.move_type == 1:
            self.standby = "standby"
            self.status = 0
            self.reset_module()
        elif self.move_type == 2:
            self.standby = "standby"
            self.status = 0
            self.reset_module()
        elif self.move_type == 3:
            return "build_master_slave_trajectory"
        else:
            self.fail_info = "Planning type not recognized"
            return "fail_up"
            
    def build_master_slave_trajectory(self):
        """
        Plan a Cartesian trajectory for a master/slave type trajectory.
        """
        
        # Arm speed
        speed = 0.01 #m/s (default for now)
        
        # Determine which motion parameters
        cur_pose = self.arm_states[2]
        goal_pose = self.goal_poses.poses[0]
        rel_pose = self.goal_poses.poses[1]
        if self.master == "left":
            master_frame = "left_" + self.eef_name
        else:
            master_frame = "right_" + self.eef_name
        
        # Create trajectories for the master and slave arm (TODO: improve)
        # arm_a_traj, timestamps = traj.create_in_place_rotation_trajectory(cur_pose,
                                                                    # goal_pose,
                                                                    # 0.02,
                                                                    # self.ref_frame)
        arm_a_traj, timestamps = traj.create_simple_move_trajectory(cur_pose,
                                                                     goal_pose,
                                                                     speed,
                                                                     self.ref_frame)
        arm_b_traj = traj.create_static_trajectory(rel_pose,
                                                   timestamps,
                                                   master_frame)
        
        # Handle the timesteps for both trajectories
        times = []
        for time in timestamps:
            time_float = time["secs"] + 0.000000001*time["nsecs"]
            time_dur = rospy.Duration(time_float)
            times.append(time_dur)
        
        # Package both trajectories into one 'TimedPoseArray' message
        self.trajectory = TimedPoseArray()
        self.trajectory.poses = arm_a_traj.poses + arm_b_traj.poses
        self.trajectory.time_from_start = times + times
        
        return True
        
    def execute_trajectory(self):
        """
        Execute the given trajectory plan.
        """
        
        # Package and publish the arm messages to the correct arms
        rospy.loginfo("Excuting trajectory...")
        self.msg = self.build_HLtoRL_msg(True, 2, self.trajectory)
        self.HLtoRL_pub.publish(self.msg)
        success = True
            
        return success
        
    def reset_module(self):
        """
        Reset the module parameters and variables to a state in which 
        a new command can be executed.
        """
        
        # Reset relevent parameters/variables
        self.executed = False
        self.new_command = False
        self.trajectory = None


if __name__ == "__main__":
    try:
        UnifiedHabitualModule()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
