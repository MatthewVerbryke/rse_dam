#!/usr/bin/env python

"""
  A script to run various other specified scripts and open them together.
  
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

import fileFunctions as fF


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
                        "sim",
                        "empty_world.launch",
                        "moveit.py",
                        "pseudoDL_v3.0.py"] # default connection valid because starting rosbridge from this file :)
        [computer_name,
         sim_or_real,
         gazebo_world,
         HL_call,
         DL_call] = fF.readArgvpieces(sys.argv,argvdefaults)
         
        # General parameters (no alternatives ATM)
        robotType = "boxbot"
        simType = "gazebo"
        some_flag = 1
        
        # Deliberative module parameters
        DLsolverUse = "None"
        DLplannerstr = "None"
        
        # Habitual module parameters (no alternatives ATM)
        arm_group = "widowx_arm"
        gripper_group = "widowx_gripper"
        ref_frame = "origin_point"
        launch_rviz = True
        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        
        # Determine parameters dependent on computer
        if (computer_name == "Friday2"):
            print ("This is the main simulation computer.")
            left_arm = True
            launch_gazebo = True
            launch_DL = True
            launch_state_pub = True
            launch_arm_command = False
            connection = "ws://192.168.0.50:9090/"
        elif (computer_name == "cameronhurd"):
            print ("This is the secondary computer.")\
            left_arm = False
            launch_gazebo = False
            launch_DL = False
            launch_state_pub = False
            launch_arm_command = True
            connection = "ws://192.168.0.51:9090/"
            
        # Determine parameters dependent on whether we are running a sim or not
        # currently, while running this file it is assumed we are running the real thing
        # or running in gazebo
        if (sim_or_real == "sim"):
            sim = True
        else:
            sim = False
                  
        # run catkin_make FIRST in the original terminal window and wait
        # 'til it's complete before doing any else! (otherwise you'll get
        # weird ROS ID error messages from the roscore and rosbridge_server
        # if the messages were changed/updated but not recompiled)
        command = ['bash', '-c', '''
                        cd %(topdir)s
                        source devel/setup.bash && catkin_make
        ''' % locals()]
            
        subprocess.call(command) # each call originates from original directory
                                 # so "cd" here is "ignored" further down :)
                                 
        
        print(" ")
        print(" ")
        print(" ")
        
        # TODO: print instructions to screen
        
        tabtitle = []
        command = []
        
        # Prepare the rosbridge server
        tabtitle.extend(["rosbridge_server (status)"])
        command.extend([''' source %(topdir)s/devel/setup.bash
                            roslaunch rosbridge_server_9090.launch
        ''' % locals()]
        
        # If we are in sim, prepare Gazebo
        if (sim_or_real == "sim"):
            tabtitle.extend(["gazebo"])
            command.extend([''' source %(topdir)s/devel/setup.bash
                                roslaunch boxbot_gazebo %(gazebo_world)s
            ''' % locals()]
            
        # Setup the Move group and RL scripts (assumes you are running an arbotix controller)
        tabtitle.extend(["movegroup/RL"])
        command.extend([''' source %(topdir)s/devel/setup.bash
                            roslaunch widowx_arm_bringup boxbot_arms_moveit.launch sim:=%(sim)s gazebo:=%(sim)s left_arm:=%(left_arm)s

        ''' % locals()]
        
        # Launch all the previous commands in a multitab terminal
        terminal = ['gnome-terminal']
        for i in range(len(command)):
            terminal.extend(['--tab', '-e', '''
                bash -c '
                    %s
                    read
                '
            ''' % (command[i],), '-t', '%s' % (tabtitle[i],)])
    
        # Launch the DL and HL scripts in a separate terminal TODO
        terminal.extend(['--tab', '-e', '''
            bash -c '
                ./DLHL_launch.py %(echo_input)d "%(Estcall)s" "%(RLcall)s" "%(HLcall)s" "%(DLcall)s" %(connection)s
                read
            '
        ''' % locals(), '-t', '%s' % ('popout gnome-terminal',)])
        
        print("All init-scripts have now been called.")
        
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        try:
            print("Exception occurred, stopping script.")
        except NameError:
            print("NameError")

        sleep(3)
