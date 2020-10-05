#!/usr/bin/env python

"""
  A ROS parameter retriever that does not use rosparam or the ROS
  parameter server. See note below.
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import os

import yaml #<-- requires PyYAML


def retrieve_params_yaml_files(file_str, dir_path):
    """
    A function to retrieve parameters from a yaml file the "hard way".
    
    NOTE: This exists because nodes launched using subprocess.call (such
    as those in 'multilaunch.py') seem to be unable to retrieve 
    parameters from the ROS parameter server in the usual manner for 
    some reason.
    
    TODO: Can the above problem be fixed?
    """
    
    # Set cwd
    home_dir = os.getcwd()
    
    # Break apart files names into separate names with file extension
    file_list = file_str.split(",")
    for i in range(0,len(file_list)):
        file_list[i] += ".yaml"
    
    try:
        
        # Go to configuration file directory
        os.chdir(dir_path)
        
        # Retrive contents from each file and store in main dictionary
        params = {}
        for file_name in file_list:
            with open(file_name) as f:
                content = yaml.load(f, Loader=yaml.FullLoader)
            params.update(content)
        
        return params
        
    finally:
        
        # Go back to starting directory
        os.chdir(home_dir)
