#!/usr/bin/env python

"""
  Deliberative layer module for a dual arm robot. 

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: IMPROVE DOCUMENTATION
"""


import os
import sys

from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import RobotTrajectory
import rospy
import tf

import adapter as adpt
import trajectories as traj

# retrieve files nessecary for websocket comms and layer comms
file_dir = sys.path[0]
sys.path.append(file_dir + "/..")
sys.path.append(file_dir + "/../..")
from communication import rse_packing as rpack
from rse_dam_msgs.msg import HLtoDL, DLtoHL, OptoDL, DLtoOp
from rss_git_lite.common import rosConnectWrapper as rC
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS


class DeliberativeModule(object):
    """
    The RSE Deliberative Module for a dual-armed robot.
    
    TODO: TEST
    """
    
    def __init__(self):

        # Initialize rospy node
        rospy.init_node("deliberative_module")
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Get relevent parameters
        self.ref_frame = sys.argv[1]
        self.left_eef_frame = sys.argv[2]
        self.right_eef_frame = sys.argv[3]
        self.connection = sys.argv[4]
        rospy.loginfo("reference frame: {}".format(self.ref_frame))
        rospy.loginfo("left end effector: {}".format(self.left_eef_frame))
        rospy.loginfo("right end effector: {}".format(self.right_eef_frame))
        rospy.loginfo("secondary computer IP address: {}".format(self.connection))
        
        # Module information setup
        self.state = "start"
        self.substate = ""
        self.status = 0
        self.fail_info = ""
        
        # Task information setup
        self.move_type = 0
        self.traj_template = 0
        self.template = ""
        self.object_position = Pose()
        self.object_goal = Pose()
        self.left_grasp_pose = Pose()
        self.right_grasp_pose = Pose()
        self.left_gap = 0.0
        self.right_gap = 0.0
        self.move_speed = 0.0
        self.lift_height = 0.0
        self.old_goal = Pose()
        
        # Trajectory information setup
        self.stamps = None
        self.obj_traj = None
        self.left_offset = []
        self.left_poses = None
        self.left_traj = RobotTrajectory()
        self.right_offset = []
        self.right_poses = None
        self.right_traj = RobotTrajectory()
        
        # Lower layer status setup
        self.left_return = HLtoDL()
        self.right_return = HLtoDL()
        
        # Create a transform listener
        self.listener = tf.TransformListener()
        rospy.sleep(1.0)        
        
        # Initialize communications for the layer
        self.comms_initialization()
        rospy.loginfo("Communcation interfaces setup")
        
        # Run main loop
        self.main()
        
    def main(self):
        """
        Main execution loop for the DL module.
        """
        
        # Run through state machine and comms updates once each loop while rospy is running
        while not rospy.is_shutdown():
            print self.state
            print self.substate
            print ""
            self.run_state_machine()
            self.publish_DLtoOp()
            rospy.sleep(0.01)
            #raw_input("Enter for next state...")
    
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down node 'deliberative_module'")
        rospy.sleep(1)

    #==== Communications ==============================================#
    
    def comms_initialization(self):
        """
        Intialize communications for the deliberative module.
        
        TODO: TEST
        """
        
        # Setup publishers
        # NOTE: For now, we assume that the left arm movegroup is on the
        #       same computer as the DL. The right arm movegroup is assumed
        #       to be on a separate computer, requiring websocket comms.
        self.DL_to_left_HL_pub = rospy.Publisher("/deliberative/to_hl", DLtoHL, queue_size=1)
        self.DL_to_right_HL_pub = rC.RosMsg('ws4py', self.connection, "pub", "/deliberative/to_hl", "rse_dam_msgs/DLtoHL", rpack.pack_DL_to_HL)
        self.DL_to_Op_pub = rospy.Publisher("/deliberative/to_op", DLtoOp, queue_size=1)
        
        # Setup subscribers
        self.left_HL_to_DL_sub = rospy.Subscriber("/left_habitual/to_dl", HLtoDL, self.left_hl_cb)
        self.right_HL_to_DL_sub = rospy.Subscriber("/right_habitual/to_dl", HLtoDL, self.right_hl_cb)
        self.Op_to_DL_sub = rospy.Subscriber("/operator/to_dl", OptoDL, self.op_cb)
    
    def left_hl_cb(self, msg):
        """
        Callback for the left arm habitual module messages.
        """
        self.left_return = msg

    def right_hl_cb(self, msg):
        """
        Callback for right arm habitual module messages.
        """
        self.right_return = msg
    
    def op_cb(self, msg):
        """
        Callback for operator messages.
        """
        
        # Extract and store info from operator message
        self.move_type = msg.move_type
        self.traj_template = msg.template
        self.object_position = msg.object_start
        self.object_goal = msg.object_goal
        self.left_grasp_pose = msg.left_grasp_pose
        self.left_gap = msg.left_gap
        self.right_grasp_pose = msg.right_grasp_pose
        self.right_gap = msg.right_gap
        self.move_speed = msg.move_speed
        self.lift_height = msg.lift_height
        
    def create_DLtoHL(self, new_cmd, move_type, poses, stamps, target_pose, command):
        """
        Create a 'DLtoHL' message
        
        TODO: TEST
        """
        
        # Fill out DL to HL message from input info
        msg = DLtoHL()
        msg.new_cmd = new_cmd
        msg.move_type = move_type
        msg.poses = poses
        msg.stamps = str(stamps)
        msg.target_pose = target_pose
        msg.command = command
        
        return msg
        
    def create_DLtoOp(self):
        """
        Create a 'DLtoOp' message
        
        TODO: TEST
        """
        
        # Fill out the DL to OP message from input info 
        msg = DLtoOp()
        msg.dl_status = self.status
        msg.right_status = self.right_return.status
        msg.left_status = self.left_return.status
        msg.fail_info = self.fail_info
        return msg
        
    def publish_DLtoOp(self):
        """
        Publish a message to the operator
        """
        
        # Create 'DLtoOP' message
        msg = self.create_DLtoOp()
        
        # Publish it
        self.DL_to_Op_pub.publish(msg)
        
    #==== State Machine ===============================================#
    
    def run_state_machine(self):
        """
        Run through the DL state machine.
        """
        
        # TODO: CREATE AN INTERRUPT-HANDLER
        
        # MAIN STATE MACHINE
        # Start
        if (self.state=="start"): 
            self.state = "standby"
        
        # Wait for the operator to send a new command
        elif (self.state=="standby"): 
            new_task = self.wait_for_command()
            if new_task:
                self.state = "check commands"
                self.old_goal = self.object_goal
                self.status = 1
            else:
                pass
        
        # Check operator goals and constraints
        elif (self.state=="check commands"):
            commands_ok = self.check_command()
            if commands_ok:
                self.state = "determine move type"
            else:
                self.state = "fail up"
                self.status = 4
        
        # Determine the required planning type
        elif (self.state=="determine move type"):
            plan_type = self.determine_move_type(self.move_type, self.traj_template)
            self.status = 2
            if (plan_type=="dual-arm"):
                self.state = "plan dual-arm"
                self.substate = "plan object trajectory"
            elif (plan_type=="left"):
                self.state = "plan left arm"
            elif (plan_type=="right"):
                self.state = "plan right arm"
            else:
                self.state = "fail up"
                self.status = 4
        
        # Plan a bimanual or coordinated movement of both arms
        elif (self.state=="plan dual-arm"):
            
            # Substate for creating an object trajectory
            if (self.substate=="plan object trajectory"):
                self.obj_traj, self.stamps = self.get_object_trajectory(self.template)
                if (self.obj_traj!=None) and (self.stamps!=None):
                    self.substate = "get offsets"
                else:
                    self.substate = "attempt error resolution"
            
            # Substate for getting end-effector offsets
            elif (self.substate=="get offsets"):
                self.left_offset = adpt.compute_eef_offset(self.object_position, self.left_return.eef_pose)
                self.right_offset = adpt.compute_eef_offset(self.object_position, self.right_return.eef_pose)
                if (self.left_offset!=[]) and (self.right_offset!=[]):
                    self.substate = "plan eef trajectories"
                else:
                    self.substate = "attempt error resolution"
            
            # Substate for getting an array of poses for the end-effector
            # trajecories. Also, send them to the HL layer to get joint 
            # trajectories back.
            elif (self.substate=="plan eef trajectories"):
                self.left_poses = adpt.adapt_arm_poses(self.obj_traj, self.left_offset)
                self.right_poses = adpt.adapt_arm_poses(self.obj_traj, self.right_offset)
                if (self.left_poses!=None) and (self.right_poses!=None):                  
                    self.substate = "plan joint trajectories"
                else:
                    self.substate = "attempt error resolution"
            
            # Substate for getting joint trajectories for both arms
            # NOTE: This requires waiting for a response from both HL
            elif (self.substate=="plan joint trajectories"):
                plans_received, error = self.get_joint_trajectory_plans()
                if not plans_received and not error:
                    pass
                elif plans_received:
                    self.substate = "check trajectories"
                else:
                    self.substate = "attempt error resolution"
                
            # Substate for checking the produced trajectories against constraints
            elif (self.substate=="check trajectories"):
                left_traj_ok = self.check_trajectory("left")
                right_traj_ok = self.check_trajectory("right")
                if left_traj_ok and right_traj_ok:
                    self.state = "execute plan"
                    self.substate = "send execution command"
                    self.status = 3
                else:
                    self.substate = "attempt error resolution"

            # Substate for trying to resolve issues with the trajectories in order to replan
            #TODO
            elif (self.substate=="attempt error resolution"):
                self.state = "fail up"
                self.substate = ""
                self.status = 4
        
        # Plan a single arm movement with the left arm
        #TODO
        elif (self.state=="plan left arm"):
            side = "left"
            plan_made = self.create_single_arm_trajectory(side)
            if plan_made:
                self.state = "execute_plan"
            else:
                self.state = "fail up"
        
        # Plan a single arm movement with the right arm
        #TODO
        elif (self.state=="plan right arm"):
            side = "right"
            plan_made = self.create_single_arm_trajectory(side)
            if plan_made:
                self.state = "execute_plan"
            else:
                self.state = "fail up"
        
        # Execute the planned trajectory(s)
        elif (self.state=="execute plan"):
            
            # Substate for sending the command to execute the current plan
            # NOTE: Keeps publishing to the HL until it gets a response 
            #       that the plan is being executed.
            if (self.substate=="send execution command"):
                executing, error = self.execute_plan()
                if not executing and not error:
                    pass
                elif executing:
                    self.substate = "monitor execution"
                    self.status = 5
                else:
                    self.substate = "fail up"
                    self.status = 7
            
            # A substate for monitoring the progress of the plan execution.
            elif (self.substate=="monitor execution"):
                finished, error = self.check_execution_status()
                if not finished and not error:
                    pass
                elif finished:
                    self.substate = "check completion"
                    self.status = 6
                else:
                    self.substate = "fail up"
                    self.status = 7
            
            # A substate to check that the move succeded
            elif (self.substate=="check completion"):
                success = self.check_plan_achievement()
                if success:
                    self.state = "standby"
                    self.substate = ""
                    self.status = 0
                    self.reset_module()
                else:
                    self.state = "fail up"
                    self.status = 7
        
        # Send message to operator on the nature of the error/failure
        elif (self.state=="fail up"):
            pass # publisher will take care of this
    
    #==== Module Behaviors ============================================#
    
    def wait_for_command(self):
        """
        Wait for the operator to send a new command
        """
        
        # check to see if the goal sent by the operator has changed
        if (self.object_goal==self.old_goal):
            return False
        else:
            return True
        
    def check_command(self):
        """
        Check and see if the plan given by the operator is at least basically
        possible.
        """
        #TODO
        return True
        
    def determine_move_type(self, op_type, template_type):
        """
        Determine the type of planning that needs to be performed. In the 
        future, this may be detemined within this function versus just being
        told what to do.
        """
        
        # Determine the overall move_type
        if (op_type==1):
            plan_type = "dual-arm"
        elif (op_type==2):
            plan_type = "left"
        elif (op_type==3):
            plan_type = "right"
        else:
            plan_type = "error"
            self.fail_info = "Planning type not recognized"
            
        # Determine what trajectory template to use, if any
        if (template_type==1):
            self.template = "none"
        elif (template_type==2):
            self.template = "simple_move"
        elif (template_type==3):
            self.template = "in-place rotation"
        else:
            self.plan_type = "error"
            self.fail_info = "Trajectory template type not recognized"
            
        return plan_type
        
    def get_joint_trajectory_plans(self):
        """
        Send data to the HL and wait until a response is received with a 
        joint trajectory for each arm
        """
        
        # Publish data
        left_msg = self.create_DLtoHL(1, 1, self.left_poses, self.stamps, Pose(), 0)
        right_msg = self.create_DLtoHL(1, 1, self.right_poses, self.stamps, Pose(), 0)
        self.DL_to_left_HL_pub.publish(left_msg)
        self.DL_to_right_HL_pub.send(right_msg)
        
        # Listen for the response that plans have been returned  
        if (self.left_return.status==1) or (self.right_return.status==1):
            plans_returned = False
            error = False
        elif (self.left_return.status==2) and (self.right_return.status==2): 
            self.left_traj = self.left_return.trajectory
            self.right_traj = self.right_return.trajectory
            plans_returned = True
            error = False
        elif (self.left_return.status==3) or (self.right_return.status==3):
            plans_returned = False
            error = True
            if (self.left_return.status==3):
                self.fail_info = str(self.left_return.fail_msg)
            elif (self.right_return.status==3):
                self.fail_info = str(self.right_return.fail_msg)
        else:
            plans_returned = False
            error = False
            
        return plans_returned, error
        
    def create_single_arm_trajectory(self, side):
        """
        
        """
        pass
        
    def get_object_trajectory(self, template):
        """
        Based on the selected trajectory plan (user specified?), get a 
        trajectory that the target object must follow.
        """
        
        # Create the object trajectory based on the template choice
        if (template=="simple_move"):
            trajectory, timesteps = traj.create_simple_move_trajectory(self.object_position, self.object_goal, self.move_speed, self.ref_frame)
        elif (template=="pick_and_place"): # NOTE: NOT READY YET
            trajectory, timesteps = traj.create_pick_and_place_trajectory(self.object_position, self.object_goal, self.move_speed, self.ref_frame)
        elif (template=="none"):
            rospy.logerr("Need to specify a trajectory template for dual-arm manipulation")
            self.fail_info = "Need to specify a trajectory template for dual-arm manipulation"
            trajectory = None
            timesteps = None
        else:
            rospy.logerr("Trajectory template type not found")
            self.fail_info = "Trajectory template type not found"
            trajectory = None
            timesteps = None
        
        return trajectory, timesteps 
        
    def lookup_frame_transform(self, origin_frame, target_frame):       
        """
        Get the transform of between two frames using tf.LookupTransform
        """   
        
        # Try to get the info on the frames from tf if given a link name
        try:
            trans, rot = self.listener.lookupTransform(target_frame, origin_frame, rospy.Time(0))
            return [trans, rot]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to recieve the transform for {} to {}".format(origin_frame, target_frame))
            self.fail_info("Failed to recieve the transform for {} to {}".format(origin_frame, target_frame))
            return "error" #change
            
    def check_trajectory(self, side):
        """
        Check to make sure the trajectory returned is safe and is 
        consistant with constraints.
        """
        #TODO
        return True
        
    def execute_plan(self):
        """
        Execute the generated plan by sending a messege to the HL layers.
        Wait for a response before ending the publishing of the command.
        """
        # Publish data
        left_msg = self.create_DLtoHL(0, 1, PoseArray(), "", Pose(), 1)
        right_msg = self.create_DLtoHL(0, 1, PoseArray(), "", Pose(), 1)
        self.DL_to_left_HL_pub.publish(left_msg)
        self.DL_to_right_HL_pub.send(right_msg)
        
        # Listen for the response that the plan is being executed
        if (self.left_return.status==4) and (self.right_return.status==4): 
            executing = True
            error = False
        elif (self.left_return.status==5) or (self.right_return.status==5):
            executing = False
            error = True
            if (self.left_return.status==5):
                self.fail_info = str(self.left_return.fail_msg)
            elif (self.right_return.status==5):
                self.fail_info = str(self.right_return.fail_msg)
        else: 
            executing = False
            error = False
            
        return executing, error
        
    def check_execution_status(self):
        """
        Monitor the execution of the plan.
        """
        print self.left_return.status
        if (self.left_return.status==6) and (self.right_return.status==6): 
            finished = True
            error = False
        elif (self.left_return.status==7) or (self.right_return.status==7):
            finished = False
            error = True
            if (self.left_return.status==7):
                self.fail_info = str(self.left_return.fail_msg)
            elif (self.right_return.status==7):
                self.fail_info = str(self.right_return.fail_msg)
        else:
            finished = False
            error = False
            
        return finished, error
        
    def check_plan_achievement(self):
        """
        Check if the plan was completed
        """
        #TODO
        return True
        
    def reset_module(self):
        """
        Reset all the module's parameters to their start values.
        """
        self.template = ""
        self.stamps = None
        self.obj_traj = None
        self.left_offset = []
        self.left_poses = None
        self.left_traj = RobotTrajectory()
        self.right_offset = []
        self.right_poses = None
        self.right_traj = RobotTrajectory()


if __name__ == "__main__":
    try:
        DeliberativeModule()
    except rospy.ROSInterruptException:
        pass

