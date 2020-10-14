#!/usr/bin/env python

"""
  A script to run various other specified scripts and open them together.

  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test
"""


import os
import subprocess
import sys 
from time import sleep

# include the workspace source directory
sys.path.append(os.getcwd() + "/..") 
from rss_git_lite.common import fileFunctions as fF


def print_instructions_to_screen():
    """
    TODO
    """
    pass
    
def prepare_rosbridge_server(top_dir):
    """
    Prepare to launch a rosbridge server
    """
        
    tab_title = []
    command = []
    
    # Rosbridge server
    tab_title.extend(["rosbridge_server"])
    command.extend([''' source %(top_dir)s/devel/setup.bash
                        cd utilities
                        roslaunch rosbridge_server_9090.launch
    ''' % locals()])
    
    return tab_title, command

def prepare_state_estimator(robot, top_dir, param_files, param_dir, launch_dir):
    """
    Prepare commands to launch all state estimation module components
    """
    
    tab_title = []
    command = []
    
    # Robot dependent nodes for pulling state information together
    tab_title.extend(["state_estimation_support"])
    command.extend([''' source %(top_dir)s/devel/setup.bash
                        cd ..
                        roslaunch %(robot)s_bringup %(robot)s_state.launch
    ''' % locals()])
    
    # State Estimator
    tab_title.extend(["state_estimation"])
    command.extend([''' source %(top_dir)s/devel/setup.bash
                        cd estimation
                        python state_estimation.py %(param_files)s %(param_dir)s
    ''' % locals()])
    
    return tab_title, command
    
def prepare_reflexive_layer(robot, side, sim, top_dir, param_files, param_dir, launch_dir):
    """
    
    """
    
    tabtitle = []
    command = []
    
    tabtitle.extend(["reflexive_module"])
    command.extend([''' source %(top_dir)s/devel/setup.bash
                        cd reflexive
                        python reflexive.py %(param_files)s %(param_dir)s %(side)s
    ''' % locals()])
    
    return tabtitle, command

if __name__ == "__main__":
    
    # Get import directory paths
    top_dir = "../.." # where toplevel stuff needs to get to
    rse_dir = "./src/rse_dam" # where we start
    
    # Storage for launch information
    groups = []
    
    # Input parameters (TODO: set in stone currently, improve)
    robot_name = "boxbot"
    sim = "true"
    side = "left"
    param_files = "arm_6dof,connections,default"
    bringup_dir = "/home/matthew/catkin_ws/src/boxbot_ros/boxbot_bringup/"
    param_dir = bringup_dir + "config"
    launch_dir = bringup_dir + "launch"
    
    # launch parameters (TODO: improve)
    launch_rosbridge = False
    launch_SEM = False
    launch_DL = False
    launch_HL = False
    launch_RL = True
    
    try:
        
        # Get the rosbridge server launch information
        if launch_rosbridge:
            tab_title, command = prepare_rosbridge_server(top_dir)
            groups.append((tab_title, command))
        
        # Get the state estimator module(s) launch information
        if launch_SEM:
            tab_title, command = prepare_state_estimator(robot_name,
                                                         top_dir,
                                                         param_files,
                                                         param_dir,
                                                         launch_dir)
            groups.append((tab_title, command))
            
        # Get the reflexive layer module(s) launch information
        if launch_RL:
            tab_title, command = prepare_reflexive_layer(robot_name,
                                                         side, 
                                                         sim,
                                                         top_dir,
                                                         param_files,
                                                         param_dir,
                                                         launch_dir)
            groups.append((tab_title, command))
        
        # Get the habitual layer module(s) launch information
        if launch_HL:
            pass #TODO
        
        # Get the deliberative layer module(s) launch information
        if launch_DL:
            pass #TODO
            
        # Launch all desired components
        for group in groups:
            terminal = ["gnome-terminal"]
            for i in range(len(group[1])):
                terminal.extend(['--tab', '-e', '''
                    bash -c '
                        %s
                        read
                    '
                ''' % (group[1][i],), '-t', '%s' % (group[0][i],)])
            #print terminal
            subprocess.call(terminal)
            sleep(3)
            
        print("All desired components have now been called.")
    
    # Handle exceptions
    except KeyboardInterrupt:
        try:
            print("Keyboard interrupt command recieved, stopping script.")
        except NameError:
            print("Name Error")
        sleep(3)
