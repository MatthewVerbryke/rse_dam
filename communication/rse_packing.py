#!/usr/bin/env python

"""
  RSE layer communications packing functions for use with rosbridge.
  
  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
"""

import sys

from packing import pack_robottrajectory, pack_pose, pack_posearray, pack_string 


def pack_HL_to_DL(hltodl):
    """
    Package "rse_dam_msgs/HLtoDL" message.
    """
    
    # Prepare message info
    status = hltodl.status
    fail_msg = hltodl.fail_msg
    rt_msg = pack_robottrajectory(hltodl.trajectory)
    
    # Package into dict
    hltodl_msg = {"status": status,
                  "fail_msg": fail_msg,
                  "trajectory": rt_msg}
                  
    return hltodl_msg
    
def pack_DL_to_HL(dltohl):
    """
    Package "rse_dam_msgs/DLtoHL" message.
    """
    
    # Prepare message info
    move_type = dltohl.move_type
    poses = pack_posearray(dltohl.poses)
    stamps = dltohl.stamps
    target_pose = pack_pose(dltohl.target_pose)
    command = dltohl.command
    
    # Package into dict
    dltohl_msg = {"status": status,
                  "move_type": move_type,
                  "poses": poses,
                  "stamps": stamps,
                  "target_pose": target_pose,
                  "command": command}
    
    return dltohl_msg
    
