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
    Prepare an to launch a rosbridge server
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

def prepare_state_estimator(top_dir):
    """
    Prepare commands to launch all state estimation module components
    """
    
    tab_title = []
    command = []
    
    # Joint State Aggagator
    tab_title.extend(["aggragator"])
    command.extend([''' source %(top_dir)s/devel/setup.bash
                        cd estimator
                        python aggagator.py
    ''' % locals()])
    
    # Robot State Publisher
    tab_title.extend(["robot_state_publisher"])
    command.extend([''' cd utilities
                        roslaunch robot_state.launch
    ''' % locals()])
    
    # State Estimator
    tab_title.extend("state_estimation")
    command.extend(['''
                        cd estimator
                        python state_estimation.py
    ''' % locals()])
    
    return tab_title, command


if __name__ == "__main__":
    
    # Get import directory paths
    top_dir = "../.." # where toplevel stuff needs to get to
    rse_dir = "./src/rse_dam" # where we start
    
    # Storage for launch information
    groups = []
    
    # Inputs (TODO)
    launch_rosbridge = True
    launch_SEM = True
    launch_DL = False
    launch_HL = False
    launch_RL = False
    
    try:
        
        # Get the rosbridge server launch information
        if launch_rosbridge:
            tab_title, command = prepare_rosbridge_server(top_dir)
            groups.append((tab_title, command))
        
        # Get the state estimator module(s) launch information
        if launch_SEM:
            tab_title, command = prepare_state_estimator(top_dir)
            groups.append((tab_title, command))
            
        # Get the reflexive layer module(s) launch information
        if launch_RL:
            pass #TODO
        
        # Get the habitual layer module(s) launch information
        if launch_HL:
            pass #TODO
        
        # Get the deliberative layer module(s) launch information
        if launch_DL:
            pass #TODO
            
        # Launch all desired components
        for group in groups:
            terminal = ["gnome_terminal"]
            for i in range(len(group[1])):
                terminal.extend(['--tab', '-e', '''
                    bash -c '
                        %s
                        read
                    '
                ''' % (group[1][i],), '-t', '%s' % (group[0][i],)])
            subprocess.call[terminal]
            sleep(9)
            
        print("All desired components have now been called.")
    
    # Handle exceptions
    except KeyboardInterrupt:
        try:
            print("Keyboard interrupt command recieved, stopping script.")
        except NameError:
            print("Name Error")
        sleep(3)

