#!/usr/bin/env python

"""
  Moveit interface submodule for RSE.

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
"""


import copy
import sys
 
from geometry_msgs.msg import PoseStamped, PoseArray
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint

from reachability import is_pose_reachable


#TODO: PASS THESE IN AS ARGUMENTS
ARM_GROUP = "widowx_arm"
GRIPPER_GROUP = "widowx_gripper"
REF_FRAME = "base_link"
LAUNCH_RVIZ = False
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


class RSEMoveItInterface(object):
    """
    A class to interface the MoveIt! motion planning framework with the 
    RSE habitual layer.
    """
    
    def __init__(self):
        """
        Initialize the move group interface for the selected planning 
        group.
        """
        
        # Get the planning group and reference frame
        # TODO: PASS IN THROUGH ARGUMENTATION
        self.planning_group = ARM_GROUP
        gripper_group = GRIPPER_GROUP
        self.ref_frame = REF_FRAME
        self.joint_names = JOINT_NAMES
        
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize rospy node
        rospy.init_node("{}_moveit_interface".format(self.planning_group), anonymous=True)
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Wait for RViz to start up, if we are running it
        if LAUNCH_RVIZ:
            rospy.sleep(10)
            
        # Initialize a move group for the main arm planning group
        self.arm = moveit_commander.MoveGroupCommander("{}".format(self.planning_group))
        
        # Initialize a move group for the gripper planning group
        self.gripper = moveit_commander.MoveGroupCommander("{}".format(gripper_group))
        
        # Get end effector link name
        self.eef_link = self.arm.get_end_effector_link()
        
        # Set the pose target reference frame
        self.target_ref_frame = self.ref_frame
        self.arm.set_pose_reference_frame(self.ref_frame)
        
        # Allow for replanning
        self.arm.allow_replanning(True)
        
        # Add tolerence to goal position and orientation
        self.arm.set_goal_position_tolerance(0.075)
        self.arm.set_goal_orientation_tolerance(0.05)
        rospy.loginfo("MoveIt! interface initialized")
        
        # Setup a connection to the 'compute_ik' service
        self.ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        self.ik_srv.wait_for_service()
        rospy.loginfo("IK service initialized")
        
        # Setup a connection to the 'compute_fk' service
        self.fk_srv = rospy.ServiceProxy("/compute_fk", GetPositionFK)
        self.fk_srv.wait_for_service()
        rospy.loginfo("FK service initialized")
        
        # Initialize a class variable for the most recently produced plan
        self.trajectory = None
        
    def plan_to_pose_goal(self, pose):
        """
        Create a trajectory plan for the group from the current pose to 
        the goal pose.
        """
        
        # Get the target pose from the function input
        target_pose = pose
        
        # Set the start state to the current state
        self.arm.set_start_state_to_current_state()

        # Set the goal pose to the target pose
        self.arm.set_pose_target(target_pose, self.eef_link)

        # Plan the trajectory
        self.trajectory = self.arm.plan()
        
    def plan_to_joint_angle_goal(self, joint_angles):
        """
        Create a trajectory plan for the group from the current pose to 
        the goal joint angle target.
        """
        
        # Clear the pose targets
        self.arm.clear_pose_targets()
        
        # Set the joint value target
        self.arm.set_joint_value_target(joint_angles)
        
        # Plan the trajectory
        self.trajectory = self.arm.plan()
        
    def plan_to_cartesian_path(self, waypoints):
        """
        Create a trajectory plan for the group from the current pose to 
        the goal joint angle target.
        """
        pass #for now
        
    def plan_to_named_pose(self, named_pose):
        """
        Create a trajectory plan for the group from the current pose to 
        the goal joint angle target.
        """
        
        # Set the named pose as the planning target
        self.arm.set_named_target(named_pose)
        
        # Plan the trajectory
        self.trajectory = self.arm.plan()
        
    def get_eef_poses_from_trajectory(self, trajectory):
        """
        For a RobotTrajectory, determine the end-effector pose at each 
        time step along the trajectory.
        
        TODO: TEST THIS FUNCTION
        """
        
        # Create the output array
        poses_out = PoseArray()
        poses_out.header.seq = copy.deepcopy(trajectory.joint_trajectory.header.seq)
        poses_out.header.stamp = rospy.Time.now()
        poses_out.header.frame_id = 1
        
        # Loop setup
        poses_tot = len(trajectory.joint_trajectory.points)
        pose_list = [None]*poses_tot
        joint_names = trajectory.joint_trajectory.joint_names      
        
        # Build array of 'PoseStamped' waypoints
        for i in range(0,poses_tot):
            
            # Get the pose using FK
            joint_states = trajectory.joint_trajectory.points[i].positions
            time = trajectory.joint_trajectory.points[i].time_from_start
            resp = self.fk_solve(joint_states, joint_names)
            
            # Put the pose into list
            pose.header.seq = i
            pose.header.stamp = time
            pose_list[i] = resp.pose_stamped
        
        # Put generated list into the PoseArray
        poses_out.poses = pose_list
        
        return poses_out
        
    def get_trajectory_from_eef_poses(self, points_array):
        """
        Given a list of time-stamped poses for the group, construct a 
        'RobotTrajectory'.
        
        TODO: ADD VELOCITY INTEPOLATION
        """
        
        # Initialize trajectory
        self.trajectory = RobotTrajectory()
        self.trajectory.joint_trajectory.header.seq = copy.deepcopy(points_array.header.seq)
        self.trajectory.joint_trajectory.header.stamp = rospy.Time.now()
        self.trajectory.joint_trajectory.header.frame_id = self.ref_frame
        self.trajectory.joint_trajectory.joint_names = self.joint_names
        
        # Loop setup
        poses_tot = len(points_array.poses)
        trajectory_points = [None]*poses_tot
        joint_positions = 
        
        # Build an array of JointTrajectoryPoint waypoints
        for i in range(0,poses_tot):

            # Check reachability
            reachable = is_pose_reachable(points_array.poses[i])
            
            # Get the joint states using IK
            resp = self.ik_solve(points_array.poses[i], joint_positions)
                        
            # Initialize JointTrajectoryPoint message
            trajectory_point = JointTrajectoryPoint()
            joint_positions = [None]*5 
            
            # Remove the gripper joints from the message 
            # TODO: better way to do this?
            for j in range(0,5):
                joint_positions[j] = resp.solution.joint_state.position[j]
        
            # Format the outputs
            trajectory_point.positions = joint_positions
            trajectory_point.time_from_start = points_array.poses[i].header.stamp
            
            # Put the joint states into the outgoing array
            trajectory_points[i] = trajectory_point
            
        # Put waypoints into the trajectory
        self.trajectory.joint_trajectory.points = trajectory_points
        
    def move_gripper(self, gripper_goal):
        """
        Open or close the gripper to a given joint angle goal.
        """
        
        # Set the joint goal for the gripper
        self.gripper.set_joint_value_target(gripper_goal)
        
        # Go to the specified position
        self.gripper.go()
        
        # Wait a second
        rospy.sleep(1)
        
    def execute_trajectory(self):
        """
        Execute the given trajectory plan.
        """
        
        # Execute the generated plan
        rospy.loginfo("Excuting trajectory...")
        self.arm.execute(self.trajectory)
                
        # Wait a second
        rospy.sleep(1)
        
    def get_target_pose(self, msg):
        """
        Callback function for the pose goal.
        """
        self.pose_command = msg
        
    def get_current_eef_pose(self):
        """
        Get the current pose of the arm eef.
        """
        pose = self.arm.get_current_pose(self.eef_link)
        
        return pose
        
    def fk_solve(self, joint_angles, link_names):
        """
        Given a set of joint angles for the robot, use forward kinematics
        to determine the end-effector pose reached with those angles

        https://github.com/uts-magic-lab/moveit_python_tools/blob/master/src/moveit_python_tools/get_fk.py
        """
        
        # Build the service request
        req = GetPositionFKRequest()
        req.header.frame_id = self.ref_frame
        req.fk_link_names = [link_names]
        req.robot_state.joint_state = joint_angles
        
        # Try to send the request to the 'compute_fk' service
        try:
            resp = self.fk_srv.call(req)
            return resp.pose_stamped
            
        except rospy.ServiceException:
            rospy.logerr("Service execption: " + str(rospy.ServiceException))
        
    def ik_solve(self, pose, feed_joint_state=[0,0,0,0,0]):
        """ 
        Given a end-effector pose, use inverse kinematics to determine the
        nessecary joint angles to reach the pose.
        
        https://github.com/uts-magic-lab/moveit_python_tools/blob/master/src/moveit_python_tools/get_ik.py
        """
        
        # Build the the service request
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.planning_group
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = feed_joint_state
        req.ik_request.avoid_collisions = True        
        req.ik_request.ik_link_name = self.eef_link
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout.secs = 1.0
        req.ik_request.timeout.nsecs = 0.0
        req.ik_request.attempts = 0

        # Try to send the request to the 'compute_ik' service
        try:
            resp = self.ik_srv.call(req)
            return resp
            
        except rospy.ServiceException:
            rospy.logerr("Service execption: " + str(rospy.ServiceException))
    
    def get_velocities(self):
        """
        TODO
        """
        pass
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        # Log shutdown
        rospy.loginfo("Shutting down node '{}_moveit_interface'".format(self.planning_group))
        
        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)
        rospy.sleep(1)

