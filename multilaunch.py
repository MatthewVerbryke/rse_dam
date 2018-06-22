#!/usr/bin/env python

"""
  A script to run various other specified scripts and open them together.
  WIP
  
  Modified from:
  https://github.com/cmcghan/rss_git_lite/blob/master/multiopen_old_combined.py
  
  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.
"""


import os
import subprocess
import sys 
from time import sleep

# include the workspace source directory
sys.path.append(os.getcwd() + "/..") 
from rss_git_lite.common import fileFunctions as fF


def printInstructionsToScreen(runScriptType):
    """ """
    pass
    
if __name__ == "__main__":
    
    # Get important directories
    topdir = "../.." # where toplevel stuff needs to get to
    cddir = "./src/rse_dam" # where we start
    
    print("Starting multilaunch.py...")
    
    try:
        print("sys.argv = " + str(sys.argv))

        # Get file argument values
        argvdefaults = ["Friday2",
                        "gazebo",
                        "empty_world.launch",
                        "moveit.py",
                        "deliberative.py"] # default connection valid because starting rosbridge from this file :)
        [computer_name,
         simulated,
         gazebo_world,
         HL_call,
         DL_call] = fF.readArgvpieces(sys.argv,argvdefaults)
         
        # General parameters (no alternatives ATM)
        robot_type = "boxbot"
        
        # Gazebo parameters
        sim_type = "gazebo"
        sim_ref_frame = "world"
        left_eef_frame = "left_wrist_2_link"
        right_eef_frame = "right_wrist_2_link"
        
        # Habitual module parameters (no alternatives ATM)
        arm_group = "widowx_arm"
        gripper_group = "widowx_gripper"
        HL_ref_frame = "origin_point"
        launch_rviz = True
        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        
        # Determine parameters dependent on computer
        if (computer_name=="Friday2"):
            print ("This is the main simulation computer.")
            left_arm = True
            launch_gazebo = True
            launch_DL = True
            launch_state_pub = True
            launch_arm_command = False
            connection = "ws://192.168.0.50:9090/"
        elif (computer_name=="cameronhurd"):
            print ("This is the secondary computer.")
            left_arm = False
            launch_gazebo = False
            launch_DL = False
            launch_state_pub = False
            launch_arm_command = True
            connection = "ws://192.168.0.51:9090/"
        
        # Determine if we are running a Gazebo simulation, running on actual
        # hardware, or only using fake drivers in RViz. If Gazebo is running,
        # sim is false due to it interfacing similar to actual hardware.
        if (simulated=="true"): #<--RViz only
            sim = True
            DL_ref_frame = HL_ref_frame
            launch_gazebo = False
            launch_state_pub = False
            gazebo_running = False
            launch_arm_command = False
        elif (gazebo_world=="real"): #<--actual hardware
            sim = False
            DL_ref_frame = HL_ref_frame
            launch_gazebo = False
            gazebo_running = False
            launch_state_pub = False
            launch_arm_command = False
        else:  #<-- Gazebo simulation
            sim = False
            gazebo_running = True
            DL_ref_frame = sim_ref_frame
                  
        # run catkin_make FIRST in the original terminal window and wait
        # 'til it's complete before doing any else! (otherwise you'll get
        # weird ROS ID error messages from the roscore and rosbridge_server
        # if the messages were changed/updated but not recompiled)
        #command = ['bash', '-c', '''
                        #cd %(topdir)s
                        #source devel/setup.bash && catkin_make
        #''' % locals()]
            
        #subprocess.call(command) # each call originates from original directory
                                 # so "cd" here is "ignored" further down :)
        
        # TODO: print instructions to screen
        
        #== TERMINAL 1 =================================================
        
        tabtitle = []
        command = []
        
        # If we are using Gazebo, prepare needed script(s)
        if launch_gazebo:
            
            # Gazebo
            tabtitle.extend(["gazebo"])
            command.extend([''' source %(topdir)s/devel/setup.bash
                                roslaunch boxbot_gazebo %(gazebo_world)s
            ''' % locals()])
        
        # Prepare the rosbridge server
        tabtitle.extend(["rosbridge_server (status)"])
        command.extend([''' source %(topdir)s/devel/setup.bash
                            roslaunch rosbridge_server_9090.launch
        ''' % locals()])
        
        terminal = ['gnome-terminal']
        for i in range(len(command)):
            terminal.extend(['--tab', '-e', '''
                bash -c '
                    %s
                    read
                '
            ''' % (command[i],), '-t', '%s' % (tabtitle[i],)])
                
        # Launch rosbridge server and (maybe) Gazebo
        subprocess.call(terminal)
        sleep(8)
        
        #== TERMINAL 2 =================================================
        
        tabtitle = []
        command = []
        
        # Setup the Move group and RL scripts (assumes you are running an arbotix controller)
        tabtitle.extend(["movegroup/RL"])
        command.extend([''' source %(topdir)s/devel/setup.bash
                            roslaunch widowx_arm_bringup boxbot_arms_moveit.launch sim:=%(sim)s gazebo:=%(gazebo_running)s left_arm:=%(left_arm)s

        ''' % locals()])
        
        # Arm state publisher
        if launch_state_pub:
            tabtitle.extend(["arm-state"])
            command.extend([''' cd communication
                                python arm_state.py %(connection)s %(left_arm)s %(robot_type)s
        ''' % locals()])
            
        # Arm command
        if launch_arm_command:
            tabtitle.extend(["arm-command"])
            command.extend([''' cd communication
                                python arm_commands.py %(connection)s %(left_arm)s %(robot_type)s
        ''' % locals()])
        
            # Wait for the state publisher on the left computer start first
            sleep(2)
        
        # Ready the new terminal
        terminal2 = ['gnome-terminal']
        for i in range(len(command)):
            terminal2.extend(['--tab', '-e', '''
                bash -c '
                    %s
                    read
                '
            ''' % (command[i],), '-t', '%s' % (tabtitle[i],)])
            
        # Launch all the previous commands in a multitab terminal
        subprocess.call(terminal2)
        sleep(9)
        
        #== TERMINAL 3 =================================================

        tabtitle = []
        command = []
        
        # Setup the Habitual module/ MoveIt interface
        tabtitle.extend(["habitual"])
        command.extend([''' cd habitual
                            python %(HL_call)s %(arm_group)s %(gripper_group)s %(HL_ref_frame)s %(left_arm)s %(connection)s
        ''' % locals()])
        
        # Ready the new terminal
        terminal3 = ['gnome-terminal']
        for i in range(len(command)):
            terminal3.extend(['--tab', '-e', '''
                bash -c '
                    %s
                    read
                '
            ''' % (command[i],), '-t', '%s' % (tabtitle[i],)])
        
        subprocess.call(terminal3)
        sleep(4)
        
        #== TERMINAL 4 =================================================
        
        tabtitle = []
        command = []
        
        # Setup the deliberative module
        if launch_DL:
            tabtitle.extend(["deliberative"])
            command.extend([''' cd deliberative
                                python %(DL_call)s %(DL_ref_frame)s %(left_eef_frame)s %(right_eef_frame)s %(connection)s
            ''' % locals()])
            
        # Ready the new terminal
        terminal4 = ['gnome-terminal']
        for i in range(len(command)):
            terminal4.extend(['--tab', '-e', '''
                bash -c '
                    %s
                    read
                '
            ''' % (command[i],), '-t', '%s' % (tabtitle[i],)])
        
        subprocess.call(terminal4)
        sleep(4)
        
        print("All init-scripts have now been called.")
    
    except KeyboardInterrupt:
        try:
            print("Keyboard interrupt command recieved, stopping script.")
        except NameError:
            print("NameError")

        sleep(3)

