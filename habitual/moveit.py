#!/usr/bin/env python

"""
  Moveit-based habitual layer module for one robotic arm in a dual-arm 
  robot.

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation
"""


import ast
import copy
import sys
import thread
 
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from sensor_msgs.msg import JointState
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint

import tactics.reachability as rech
import tactics.correction as crct

# retrive files nessecary for websocket comms
file_dir = sys.path[0]
sys.path.append(file_dir + "/..")
sys.path.append(file_dir + "/../..")
from common import params
from communication import rse_packing as rpack
from rse_dam_msgs.msg import HLtoDL, DLtoHL, HLtoRL, RLtoHL, DualArmState
from rss_git_lite.common import rosConnectWrapper as rC
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS


class MoveItHabitualModule(object):
    """
    RSE habitual layer for a dual-armed robot, using a MoveIt interface.
    """
    
    def __init__(self):
        """
        Initialize the the HL module with a move group interface for the
        selected planning group.
        """
        
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Get command line arguments
        param_files = sys.argv[1]
        param_dir = sys.argv[2]
        self.side = sys.argv[3]
        
        # Get kinematic parameters from retrieved files
        param_dict = params.retrieve_params_yaml_files(param_files, param_dir)
        self.robot = param_dict["robot"]
        self.planning_group = self.side + param_dict["planning_group"]
        self.gripper_group = self.side + param_dict["gripper_group"]
        self.ref_frame = "world" #TODO: tie to input arguments?
        arm_joints = param_dict["joints"]["arm"]
        self.joint_names = ["{}_{}_joint".format(self.side, j) for j in arm_joints]
        
        # Get ip connection for other computer (TODO: change for more than one connection)
        connections = param_dict["connections"]
        if self.side == "left":
            self.local_ip = connections["main"]
            self.connection = connections["main"]
        else:
            self.local_ip = connections["right_arm"]
            self.connection = connections["main"]
        
        # Print parameters to screen
        rospy.loginfo("Side: {}".format(self.side))
        rospy.loginfo("Planning group: {}".format(self.planning_group))
        rospy.loginfo("Gripper planning group: {}".format(self.gripper_group))
        rospy.loginfo("Reference frame: {}".format(self.ref_frame))
        rospy.loginfo("Joints: {}".format(self.joint_names))
        rospy.loginfo("Secondary computer IP address: {}".format(self.connection))
        
        # Initialize rospy node
        rospy.init_node("{}_habitual_module".format(self.side))
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
              
        # Initialize a move group for the arm and end effector planning groups
        self.arm = moveit_commander.MoveGroupCommander("{}".format(self.planning_group))
        self.gripper = moveit_commander.MoveGroupCommander("{}".format(self.gripper_group))
        
        # Get/set robot parameters
        self.eef_link = self.side + "_" + param_dict["eef_link"]
        self.target_ref_frame = self.ref_frame
        self.arm.set_pose_reference_frame(self.ref_frame)
        self.correction_needed = False
        
        # Module information setup
        self.state = "start"
        self.status = 0
        self.fail_info = ""
        self.increment = 0
               
        # Task information setup
        self.new_command = 0
        self.move_type = 0
        self.start_pose = Pose()
        self.goal_pose = Pose()
        self.command =0
        self.executed = False
        
        # Trajectory information setup
        self.current_pose = PoseStamped()
        self.stamps = ""
        self.pose_traj = PoseArray()
        self.trajectory = RobotTrajectory()
        
        # Status setup
        self.dl_return = DLtoHL()
        self.rl_return = int()
        
        # Planning information setup
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_goal_joint_tolerance(0.001)
        rospy.loginfo("MoveIt! interface initialized")
        
        # Setup a connection to the 'compute_ik' service
        self.ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        self.ik_srv.wait_for_service()
        rospy.loginfo("IK service initialized")
        
        # Setup a connection to the 'compute_fk' service
        self.fk_srv = rospy.ServiceProxy("/compute_fk", GetPositionFK)
        self.fk_srv.wait_for_service()
        rospy.loginfo("FK service initialized")
        
        # Initialize communications
        self.comms_initialization()
        rospy.loginfo("Communcation interfaces setup")
        
        # Run main loop
        self.main()
        
    def main(self):
        """
        Main execution loop for the HL module
        """
        
        r = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            #print(self.state)
            self.run_state_machine()
            self.publish_HLtoDL()
            r.sleep()
            #raw_input("Enter for next state...")
            
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

    #==== Communications ==============================================#
    
    def comms_initialization(self):
        """
        Intialize communications for the MoveIt interface.
        """
        
        # Setup publishers
        # NOTE: For now, we assume that the left arm movegroup is on the
        #       same computer as the DL. The right arm movegroup is assumed
        #       to be on a separate computer, requiring websocket comms.
        if (self.side=="left"):
            self.HL_to_DL_pub = rospy.Publisher("left_HLtoDL", HLtoDL, queue_size=1)
        elif (self.side=="right"):
            self.HL_to_DL_pub = rC.RosMsg('ws4py', "ws://"+self.connection+":9090/", "pub", "right_HLtoDL", "rse_dam_msgs/HLtoDL", rpack.pack_HL_to_DL)
        self.HL_to_RL_pub = rospy.Publisher("HLtoRL", HLtoRL, queue_size=1)
        
        # Setup subscribers
        self.DL_to_HL_sub = rospy.Subscriber("{}_DLtoHL".format(self.side), DLtoHL, self.dl_to_hl_cb)
        self.RL_to_HL_sub = rospy.Subscriber("RLtoHL", RLtoHL, self.rl_to_hl_cb)
        self.joint_state_sub = rospy.Subscriber("{}/{}_arm/joint_states".format(self.robot, self.side),
                                                JointState,
                                                self.joint_state_cb)

    def dl_to_hl_cb(self, msg):
        """
        Callback for deliberative layer msgs.
        """
        
        # Extract and store info from deliberative message
        self.lock.acquire()
        self.new_command = msg.new_cmd
        self.move_type = msg.move_type
        self.pose_traj = msg.poses
        self.stamps = msg.stamps
        self.goal_pose = msg.target_pose
        self.command = msg.command
        
        # Store last response
        self.dl_return = msg
        self.lock.release()
        
    def joint_state_cb(self, msg):
        """
        Callback for arm state msgs.
        """
        
        # Extract current joint state
        self.lock.acquire()
        self.cur_joint_state = msg
        self.lock.release()
        
    def rl_to_hl_cb(self, msg):
        """
        Callback function for messages from the reflexive layer.
        """

        #Extract information from message
        self.lock.acquire()
        self.rl_status = msg.status
        self.lock.release()
        
    def build_HLtoRL_msg(self, is_new, ctrl_type, trajectory):
        """
        Build a 'HLtoDL' message
        """
        
        # Fill out the HLtoRL message from input info
        msg = HLtoRL()
        msg.new_command = is_new
        msg.ctrl_type = ctrl_type
        msg.trajectory = trajectory
        
        return msg       
        
    def create_HLtoDL(self):
        """
        Create a 'HLtoDL' message.
        """
        
        # Fill out the HL to DL message from input info
        msg = HLtoDL()
        msg.status = self.status
        msg.fail_msg = self.fail_info
        msg.eef_pose = self.current_pose
        msg.trajectory = self.trajectory
        msg.recieved_msg = self.dl_return
        
        return msg
        
    def publish_HLtoDL(self):
        """
        Publish a message to the deliberative module.
        """
        
        # Create 'DLtoOP' message
        msg = self.create_HLtoDL()
        
        # Publish it
        if (self.side=="left"):
            self.HL_to_DL_pub.publish(msg)
        elif (self.side=="right"):
            self.HL_to_DL_pub.send(msg)
    
    #==== State Machine ===============================================#
    
    def run_state_machine(self):
        """
        Run throught the HL state machine
        
        TODO: create an interrupt handler
        """
        
        # Main State Machine
        # Start
        if (self.state=="start"):
            self.state = "standby"
            self.status = 0
            
        # Wait for commands from the DL, publish current eef pose while doing so
        elif (self.state=="standby"):
            self.status = 0
            new_command = self.wait_for_command()
            if new_command:
                self.state = "check reachability"
                self.new_command = 0
                self.status = 1
        
        # Check the reachability of the pose(s) given by the DL
        elif (self.state=="check reachability"):
            goal_reachable = True #self.check_goal_reachability()
            if goal_reachable:
                self.state = self.determine_action()
            else:
                self.state = "fail up"
                self.status = 3
                
        # TODO: Make a plan for the gripper to close or open
        
        # Make a joint trajectory plan for the arm from given poses and timestamps
        elif (self.state=="assemble plan"):
            trajectory_assembled = self.get_trajectory_from_eef_poses(self.pose_traj, self.stamps)
            if trajectory_assembled:
                self.state = "recieve execution command"
                cmd_recieved, error = self.wait_for_execution()
                self.status = 2
            else:
                self.state = "fail up"
                self.status = 3
        
        # TODO: Make a joint trajectory plan using MoveIt planning interface
        
        # Send plan to DL and wait to recieve an execution command
        elif (self.state=="recieve execution command"):
            cmd_recieved, error = self.wait_for_execution()
            if not cmd_recieved and not error:
                pass
            elif cmd_recieved and not error:
                self.state = "execute plan"
                self.status = 4
            else:
                self.state = "fail up"
                self.status = 5
        
        # Execute the planned trajectory/movement
        elif (self.state=="execute plan"):
            if not self.executed:
                success = self.execute_trajectory()
                self.executed = True
                if success:
                    self.status = 6
                else:
                    self.state = "fail up"
                    self.status = 7
                    
            # Let the DL know the HL is done
            # FIXME: a better way to do this?
            elif self.executed and (self.status==6):
                if self.rl_status!=2:
                    pass
                else:
                    self.state = "standby"
                    self.status = 0
                    self.reset_module()

        # Send message to DL on the nature of the error/failure
        elif (self.state=="fail up"):
            pass # publisher will take care of this
    
    #==== Module Behaviors ============================================#
    
    def wait_for_command(self):
        """
        Wait for the DL to send down a new goal.
        """
        
        # Check the new command flag to see if this is a new command
        if (self.new_command==0):
            return False
        elif (self.new_command==1):
            return True
            
    def check_goal_reachability(self):
        """
        Determine if given pose(s) are reachable by the arm.
        
        TODO: Add checks for gripper planning and single arm planning
        """
        
        # Passing on gripper planning for now 
        if (self.move_type==0):
            return True
        
        # Check each waypoint in the trajectory to see if it is reachable
        elif (self.move_type==1):
            for pose in self.pose_traj.poses:
                reachable = rech.is_pose_reachable(pose, self.side)
                if reachable:
                    pass
                else:
                    self.fail_info = "The pose {} is not reachable by the {} arm.".format(pose, self.side)
                    return False # <- FIXME: somewhat crude, replace?
            return True
            
        # Passing on RRT planning for now
        elif (self.move_type==2):
            return True
        
    def determine_action(self):
        """
        Determine what type of planning state to transition to based on DL
        command.
        """
        
        if (self.move_type==0):
            return "plan gripper move"
        elif (self.move_type==1):
            return "assemble plan"
        elif (self.move_type==2):
            return "make plan" 
        else:
            self.fail_info = "Planning type not recognized"
            return "fail up"
    
    def plan_to_target(self, goal):
        """
        Condensed planning function for several allowed target types: joint
        state, pose, named_target
        """
        
        # Setup
        self.arm.set_start_state_to_current_state()
        self.arm.clear_pose_targets()
        
        # Determine nessecary setup action from goal type
        if (type(goal)==Pose):
            self.arm.set_pose_target(goal, self.eef_link)
        elif(type(goal)==list):
            self.arm.set_joint_value_target(goal)
        elif(type(goal)==str):     
            self.arm.set_named_target(goal)
        
        # Plan the trajectory
        self.trajectory = self.arm.plan()
        
    def get_eef_poses_from_trajectory(self, trajectory):
        """
        For a RobotTrajectory, determine the end-effector pose at each 
        time step along the trajectory.
        
        TODO: test this function
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
        
    def get_trajectory_from_eef_poses(self, points_array, timestamp_string):
        """
        Given a list of poses and a list of corrosponding timestamps for 
        the group, construct a 'RobotTrajectory'.
        """
        
        # Initialize trajectory (TODO: improve timing)
        self.trajectory = RobotTrajectory()
        self.trajectory.joint_trajectory.header.seq = copy.deepcopy(points_array.header.seq)
        self.trajectory.joint_trajectory.header.stamp = rospy.Time(0.0)
        self.trajectory.joint_trajectory.header.frame_id = self.ref_frame
        self.trajectory.joint_trajectory.joint_names = self.joint_names
        
        # Loop setup (TODO: fix this whole section)
        poses_tot = len(points_array.poses)
        trajectory_points = [None]*poses_tot
        feed_joint_state = [0.0]*(len(self.joint_names))
        time_stamps = ast.literal_eval(timestamp_string)
        
        # Build an array of JointTrajectoryPoint waypoints
        for i in range(0,poses_tot):
            
            # Create PoseStamped for the current Pose (GetPositionIK requires PoseStamped)
            pose_req = PoseStamped()
            pose_req.header.seq = i
            pose_req.header.stamp.secs = time_stamps[i]["secs"]
            pose_req.header.stamp.nsecs = time_stamps[i]["nsecs"]
            pose_req.header.frame_id = self.ref_frame
            pose_req.pose = points_array.poses[i]
            
            # Get the joint states using IK
            resp = self.ik_solve(pose_req, feed_joint_state)
            if (resp.error_code.val!=1):
                rospy.logerr("IK solver failed on pose {} with error code {}".format(pose_req.pose, resp.error_code.val))
                self.fail_info = "IK solver failed on pose {} with error code {}".format(pose_req.pose, resp.error_code.val)
                return False
            
            # Initialize JointTrajectoryPoint message
            trajectory_point = JointTrajectoryPoint()
            
            # Filter out joint messages
            # FIXME: better way to do this?
            resp_pose = resp.solution.joint_state
            joint_positions = []
            for name in self.joint_names:
                idx = resp_pose.name.index(name)
                joint_positions.append(resp_pose.position[idx])
                
            # Wrist joint correction for the WidowX arm
            if self.correction_needed:
                fk_resp = self.fk_solve(joint_positions, self.eef_link)
                roll_angle = crct.correct_wrist_angle(fk_resp.pose_stamped[0].pose, points_array.poses[i])
                joint_positions[4] = roll_angle
        
            # Format the outputs
            trajectory_point.positions = joint_positions
            trajectory_point.time_from_start.secs = time_stamps[i]["secs"]
            trajectory_point.time_from_start.nsecs = time_stamps[i]["nsecs"]
            
            # Put the joint states into the outgoing array
            trajectory_points[i] = trajectory_point
            
            # Set current joint positions as the feed to the next waypoint
            feed_joint_state = joint_positions
            
        # Put waypoints into the trajectory
        self.trajectory.joint_trajectory.points = trajectory_points
        
        return True
        
    def move_gripper(self, gripper_goal):
        """
        Open or close the gripper to a given joint angle goal.
        
        TODO: Rework
        """
        
        # Set the joint goal for the gripper
        self.gripper.set_joint_value_target(gripper_goal)
        
        # Go to the specified position
        self.gripper.go()
        
        # Wait a second
        rospy.sleep(1)
        
    def wait_for_execution(self):
        """
        Keep sending plan to the deliberative layer until a response is 
        recieved.
        """
            
        # Listen for confirmation that the execution command has been recieved
        if (self.command==0):
            command_recieved = False
            error = False
        elif (self.command==1):
            command_recieved = True
            error = False
        else:
            command_recieved = False
            error = True
            self.fail_info = "'command' value not recognized"
            
        return command_recieved, error
        
    def execute_trajectory(self):
        """
        Execute the given trajectory plan.
        
        TODO: Rework?
        """
        
        self.trajectory.joint_trajectory.header.stamp = rospy.Time.now() + rospy.Duration(3.0)
        msg = self.build_HLtoRL_msg(True, 1,  self.trajectory)
        
        # Execute the generated plan
        rospy.loginfo("Excuting trajectory...")
        #success = self.arm.execute(self.trajectory)
        i = 0
        while i < 5:
            i += 1
            self.HL_to_RL_pub.publish(msg)
        
        # If it failed tell the Op and DL so
        #if not success:
            #self.fail_info = "Execution of the trajectory plan has failed"
            
        return True
        
    def reset_module(self):
        """
        Reset all the module's parameters to their start values.
        """
        self.increment = 0
        self.new_command = 0
        self.move_type = 0
        self.start_pose = Pose()
        self.goal_pose = Pose()
        self.command = 0
        self.executed = False
        self.stamps = ""
        self.pose_traj = PoseArray()
        self.trajectory = RobotTrajectory()
        rospy.loginfo("Trajectory successfully executed")
        
    def get_current_eef_pose(self):
        """
        Get the current pose of the arm eef.
        """
        self.current_pose = self.arm.get_current_pose(self.eef_link)
        
    #==== ROS Services ================================================#
        
    def fk_solve(self, joint_angles, link_names):
        """
        Given a set of joint angles for the robot, use forward kinematics
        to determine the end-effector pose reached with those angles

        CRED: https://github.com/uts-magic-lab/moveit_python_tools/blob/master/src/moveit_python_tools/get_fk.py
        """
        
        # Build joint state message
        joint_state = JointState()
        joint_state.header.frame_id = self.ref_frame
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = joint_angles
        
        # Build the service request
        req = GetPositionFKRequest()
        req.header.frame_id = self.ref_frame
        req.fk_link_names = [link_names]
        req.robot_state.joint_state = joint_state
        
        # Try to send the request to the 'compute_fk' service
        try:
            resp = self.fk_srv.call(req)
            return resp
            
        except rospy.ServiceException:
            rospy.logerr("Service execption: " + str(rospy.ServiceException))
        
    def ik_solve(self, pose, feed_joint_state):
        """ 
        Given a end-effector pose, use inverse kinematics to determine the
        nessecary joint angles to reach the pose.
        
        CRED: https://github.com/uts-magic-lab/moveit_python_tools/blob/master/src/moveit_python_tools/get_ik.py
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


if __name__ == "__main__":
    try:
        MoveItHabitualModule()
    except rospy.ROSInterruptException:
        pass

