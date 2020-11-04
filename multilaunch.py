#!/usr/bin/env python

"""
  A script to run various other specified scripts and open them together.

  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
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
    
def prepare_rosbridge_server(TOP_DIR):
    """
    Prepare to launch a rosbridge server
    """
        
    tab_title = []
    command = []
    
    # Rosbridge server
    tab_title.extend(["rosbridge_server"])
    command.extend([''' source %(TOP_DIR)s/devel/setup.bash
                        cd utilities
                        roslaunch rosbridge_server_9090.launch
    ''' % locals()])
    
    return tab_title, command

def prepare_state_estimator(robot, param_files, param_dir, TOP_DIR):
    """
    Prepare commands to launch all state estimation module components
    """
    
    tab_title = []
    command = []
    
    # Robot dependent nodes for pulling state information together
    tab_title.extend(["state_estimation_support"])
    command.extend([''' source %(TOP_DIR)s/devel/setup.bash
                        cd ..
                        rosparam set /use_sim_time true
                        roslaunch %(robot)s_bringup %(robot)s_state.launch
    ''' % locals()])
    
    # State Estimator
    tab_title.extend(["state_estimation"])
    command.extend([''' source %(TOP_DIR)s/devel/setup.bash
                        cd estimation
                        rosparam set /use_sim_time true
                        python state_estimation.py %(param_files)s %(param_dir)s
    ''' % locals()])
    
    return tab_title, command
    
def prepare_reflexive_layer(side, param_files, param_dir, TOP_DIR):
    """
    Prepare commands to launch all reflexive layer module components for
    the dual arm setup
    """
    
    tabtitle = []
    command = []
    
    if side == None:
        
        tabtitle.extend(["reflexive_module"])
        command.extend([''' source %(TOP_DIR)s/devel/setup.bash
                            cd reflexive
                            rosparam set /use_sim_time true
                            python reflexive_unified.py %(param_files)s %(param_dir)s
        ''' % locals()])
        
        
    else:

        tabtitle.extend(["{}_reflexive_module".format(side)])
        command.extend([''' source %(TOP_DIR)s/devel/setup.bash
                            cd reflexive
                            rosparam set /use_sim_time true
                            python reflexive.py %(param_files)s %(param_dir)s %(side)s
        ''' % locals()])
    
    return tabtitle, command
    
def prepare_habitual_layer(side, param_files, param_dir, TOP_DIR, robot):
    """
    Prepare commands to launch all habitual layer module components for
    the dual arm setup.
    """
    
    tabtitle = []
    command = []
    
    if side == None:
        
        tabtitle.extend(["habitual_module"])
        command.extend([''' source %(TOP_DIR)s/devel/setup.bash
                            cd habitual
                            rosparam set /use_sim_time true
                            python habitual_unified.py %(param_files)s %(param_dir)s
        ''' % locals()])
        
        
    else:
    
        tabtitle.extend(["movegroup_node"])
        command.extend([''' source %(TOP_DIR)s/devel/setup.bash
                            roslaunch %(robot)s_bringup boxbot_moveit.launch
        ''' % locals()])
        
        tabtitle.extend(["{}_habitual_module".format(side)])
        command.extend([''' source %(TOP_DIR)s/devel/setup.bash
                            cd habitual
                            rosparam set /use_sim_time true
                            python moveit.py %(param_files)s %(param_dir)s %(side)s
        ''' % locals()])
    
    return tabtitle, command
    
def prepare_deliberative_layer(param_files, param_dir, TOP_DIR):
    """
    
    """
    
    tabtitle = []
    command = []
        
    tabtitle.extend(["deliberative_module"])
    command.extend([''' source %(TOP_DIR)s/devel/setup.bash
                        cd deliberative
                        python deliberative.py %(param_files)s %(param_dir)s
    ''' % locals()])
    
    return tabtitle, command
        

if __name__ == "__main__":
    
    # Get command line inputs
    launch_DL = bool(int(sys.argv[1]))
    launch_left_HL = bool(int(sys.argv[2]))
    launch_right_HL = bool(int(sys.argv[3]))
    launch_unified_HL = bool(int(sys.argv[4]))
    launch_left_RL = bool(int(sys.argv[5]))
    launch_right_RL = bool(int(sys.argv[6]))
    launch_unified_RL = bool(int(sys.argv[7]))
    launch_SEM = bool(int(sys.argv[8]))
    launch_rosbridge = bool(int(sys.argv[9]))
    
    # Storage for launch information
    groups = []
    
    # Global Variables which shouldn't change
    TOP_DIR = "../.." # where toplevel stuff needs to get to
    RSE_DIR = "./src/rse_dam" # where we start
    
    # Input parameters (TODO: "set-in-stone" currently, improve)
    robot = "boxbot"
    sim = "true"
    param_files = "arm_6dof,connections,default"
    bringup_dir = "/home/matthew/catkin_ws/src/boxbot_ros/boxbot_bringup/"
    param_dir = bringup_dir + "config"
    launch_dir = bringup_dir + "launch"
    
    #=== PREPARE TERMINALS SCRIPTS ====================================#
    try:
        
        #--- ROSBRIDGE SERVER -----------------------------------------#
        if launch_rosbridge:
            tab_title, command = prepare_rosbridge_server(TOP_DIR)
            groups.append((tab_title, command))
        
        #--- STATE ESTIMATION MODULE ----------------------------------#
        if launch_SEM:
            tab_title, command = prepare_state_estimator(robot,
                                                         param_files,
                                                         param_dir,
                                                         TOP_DIR)
            groups.append((tab_title, command))
            
        #--- REFLEXIVE LAYER ------------------------------------------#
        # Left
        if launch_left_RL:
            tab_title, command = prepare_reflexive_layer("left",
                                                         param_files,
                                                         param_dir,
                                                         TOP_DIR)
            groups.append((tab_title, command))
        
        # Right
        if launch_right_RL:
            tab_title, command = prepare_reflexive_layer("right",
                                                         param_files,
                                                         param_dir,
                                                         TOP_DIR)
            groups.append((tab_title, command))
        
        # Single/Unified
        if launch_unified_RL:
            tab_title, command = prepare_reflexive_layer(None,
                                                         param_files,
                                                         param_dir,
                                                         TOP_DIR)
            groups.append((tab_title, command))
        
        #--- HABITUAL LAYER -------------------------------------------#
        # Warn user about Moveit preemptability
        if launch_left_HL and launch_right_HL:
            print "WARNING: two movegroups on the same ROS master will preempt each other when running the same services simulatneously"
        
        # Left
        if launch_left_HL:
            tab_title, command = prepare_habitual_layer("left",
                                                        param_files,
                                                        param_dir,
                                                        TOP_DIR,
                                                        robot)
            groups.append((tab_title, command))
        
        # Right
        if launch_right_HL:
            tab_title, command = prepare_habitual_layer("right",
                                                        param_files,
                                                        param_dir,
                                                        TOP_DIR,
                                                        robot)
            groups.append((tab_title, command))
        
        # Single/Unified
        if launch_unified_HL:
            tab_title, command = prepare_habitual_layer(None,
                                                        param_files,
                                                        param_dir,
                                                        TOP_DIR,
                                                        robot)
            groups.append((tab_title, command))
        
        #--- DELIBERATIVE LAYER ---------------------------------------#
        if launch_DL:
            tab_title, command = prepare_deliberative_layer(param_files,
                                                            param_dir,
                                                            TOP_DIR)
            groups.append((tab_title, command))
            
        #==============================================================#
        
        # Launch all the prepared components
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
        sleep(2)
