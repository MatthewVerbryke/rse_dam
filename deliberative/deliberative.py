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

import rospy
import tf

import adapter
import trajectories

# retrieve files nessecary for websocket comms
file_dir = sys.path[0]
sys.path.append(file_dir + '/../..')
from rse_dam.communication import packing
from rse_dam.rse_dam_msgs.msg import HLtoDL, DLtoHL, OptoDL, DLtoOp
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
        
        # Get relevent parameters
        # TODO: CHANGE TO ARGS
        self.ref_frame = "world"
        self.left_eef_frame = "left_wrist_2_link"
        self.right_eef_frame = "right_wrist_2_link"
        
        # Module information setup
        self.state = "start"
        self.substate = ""
        self.status = 1 #TODO
        self.fail_info = None
        self.execute = False
        
        # Task information setup
        self.move_type = 0
        self.object_position = None
        self.object_goal = None
        self.left_grasp_pose = None
        self.right_grasp Pose = None
        self.left_gap = 0.0
        self.right_gap = 0.0
        self.move_speed = 0.0
        self.lift_height = 0.0
        
        # Trajectory information setup
        self.left_traj = None
        self.left_stamps = None
        self.right_traj = None
        self.right_stamps = None
        
        # Lower layer status setup
        self.left_status = None
        self.right_status = None
        
        # Create a transform listener
        self.listener = tf.TransformListener()
        rospy.sleep(1.0)        
        
        # Initialize communications for the layer
        self.comms_initialization()
        rospy.loginfo("Communcation interfaces setup")
        
    def main(self):
        """
        Main execution loop for the DL module.
        """
        
        # Run through state machine and comms updates while rospy is running
        while not rospy.is_shutdown():
            pass
    
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
        self.DL_to_right_HL_pub = rC.RosMsg('ws4py', self.connection, "pub", "/deliberative/to_hl", "DLtoHL", self.pack_DLtoHL)
        self.DL_to_Op_pub = rospy.Publisher("/deliberative/to_hl", DLtoHl, queue_size=1)
        
        # Setup subscribers
        self.left_HL_to_DL_sub = rospy.Subscriber("/left_habitual/to_dl", HLtoDL, self.left_hl_cb)
        self.right_HL_to_DL_sub = rospy.Subscriber("/right_habitual/to_dl", HLtoDL, sefl.right_hl_cb)
        self.Op_to_DL_sub = rospy.Subscriber("/Operator/to_dl", OptoDL, self.op_cb)
    
    def left_hl_cb(self, msg):
        """
        Callback for the left arm habitual module messages.
        """
        return msg
    
    def right_hl_cb(self, msg):
        """
        Callback for right arm habitual module messages.
        """
        return msg    
    
    def op_cb(self, msg):
        """
        Callback for operator messages.
        """
        return msg    
        
    #==== State Machine ===============================================#
    
    def run_state_machine(self):
        """
        Run through the DL state machine.
        
        TODO: TEST
        """
        
        # TODO: CREATE AN INTERRUPT-HANDLER
        
        # MAIN STATE MACHINE
        # Start (not nessecary/remove?)
        if (self.state=="start"): 
            self.state=="standby"
        
        # Wait for the operator to send a new command
        elif (self.state=="standby"): 
            new_task = self.wait_for_command()
            if new_task:
                self.state = "check commands"
            else:
                pass
        
        # Check operator goals and constraints
        elif (self.state=="check commands"):
            commands_ok = self.check_command()
            if commands_ok:
                self.state = "determine move type"
            else:
                self.state = "fail up"
        
        # Determine the required planning type
        elif (self.state=="determine move type"):
            plan_type = self.determine_move_type()
            if (plan_type=="dual-arm"):
                self.state = "plan dual-arm"
            elif (plan_type=="left arm"):
                self.state = "plan left arm"
            elif (plan_type=="right arm"):
                self.state = "plan right arm"
            else:
                self.state = "fail up"
        
        # Plan a bimanual or coordinated movement of both arms
        elif (self.state=="plan dual-arm"):
            plan_made = self.create_dual_arm_trajectory()
            if plan_made:
                self.state = "execute_plan"
            else:
                self.state = "fail up"
        
        # Plan a single arm movement with the left arm
        elif (self.state=="plan left arm"):
            side = "left"
            plan_made = self.create_single_arm_trajectory(side)
            if plan_made:
                self.state = "execute_plan"
            else:
                self.state = "fail up"
        
        # Plan a single arm movement with the right arm
        elif (self.state=="plan right arm"):
            side = "right"
            plan_made = self.create_single_arm_trajectory(side)
            if plan_made:
                self.state = "execute_plan"
            else:
                self.state = "fail up"
        
        # Execute the planned trajectory(s)
        elif (self.state=="execute plan"):
            if not self.executing:
                error = self.execute_plan()
                self.executing = True
            else:
                finished, error = check_execution_status()
                if not finished:
                    pass
                else:
                    self.executing = False
                    self.state = "standby"
            if error:
                self.state = "fail up"
        
        # Send message to operator on the nature of the error/failure
        elif (self.state=="fail up"):
            self.send_failure_info()
    
    #==== Module Behaviors ============================================#
    
    def wait_for_command(self):
        """
        
        """
        pass
        
    def check_command(self):
        """
        
        """
        pass
        
    def determine_move_type(self):
        """
        
        """
        pass
        
    def create_dual_arm_trajectory(self):
        """
        
        """
        pass
        
    def create_single_arm_trajectory(self, side):
        """
        
        """
        pass
        
    def get_object_trajectory(self, template):
        """
        Based on the selected trajectory plan (user specified?), get a 
        trajectory that the target object must follow.
        
        TODO: TEST
        """
        
        # Create the object trajectory based on the template choice
        if (template=="simple move"):
            trajectory, timesteps = create_simple_move_trajectory(self.start_pose, self.goal_pose, self.move_speed, self.ref_frame)
        elif (template=="pick_and_place"): # NOTE: NOT READY YET
            trajectory = create_pick_and_place_trajectory(self.start_pose, self.goal_pose, self.move_speed, self.ref_frame)
        else:
            rospy.logerr("Trajectory type not found") # change
        
        return trajectory, timesteps 
        
    def lookup_frame_transform(self, origin_frame, target_frame):       
        """
        Get the transform of between two frames using tf.LookupTransform
        
        TODO: TEST 
        """   
        
        # Try to get the info on the frames from tf if given a link nam
        try:
            trans, rot = self.listener.lookupTransform(target_frame, origin_frame, rospy.Time(0))
            return [trans, rot]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to recieve the transform for {} to {}".format(origin_frame, target_frame)) #change
            return "error" #change
        
    def execute_plan(self):
        """
        
        """
        pass
        
    def check_execution_status(self):
        """
        
        """
        pass
        
    def send_failure_info(self):
        """
        
        """
        pass


if __name__ == "__main__":
    try:
        DeliberativeModule()
    except rospy.ROSInterruptException:
        pass

