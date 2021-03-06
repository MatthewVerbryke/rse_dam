#!/usr/bin/env python

"""
  RSE layer communications packing functions for use with rosbridge.
  
  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation
"""

import sys

from packing import *

def pack_HL_to_DL(hltodl):
    """
    Package "rse_dam_msgs/HLtoDL" message.
    """
    
    # Prepare message info
    status = hltodl.status
    fail_msg = hltodl.fail_msg
    eef_pose = pack_posestamped(hltodl.eef_pose)
    rt_msg = pack_robottrajectory(hltodl.trajectory)
    copy_msg = pack_DL_to_HL(hltodl.recieved_msg)
    
    # Package into dict
    hltodl_msg = {"status": status,
                  "eef_pose": eef_pose,
                  "fail_msg": fail_msg,
                  "trajectory": rt_msg,
                  "recieved_msg": copy_msg}
                  
    return hltodl_msg
    
def pack_DL_to_HL(dltohl):
    """
    Package "rse_dam_msgs/DLtoHL" message.
    """
    
    # Prepare message info
    new_cmd = dltohl.new_cmd
    move_type = dltohl.move_type
    poses = pack_posearray(dltohl.poses)
    stamps = dltohl.stamps
    target_pose = pack_pose(dltohl.target_pose)
    command = dltohl.command
    
    # Package into dict
    dltohl_msg = {"new_cmd": new_cmd,
                  "move_type": move_type,
                  "poses": poses,
                  "stamps": stamps,
                  "target_pose": target_pose,
                  "command": command}
    
    return dltohl_msg
    
def pack_dual_arm_state(DAS):
    """
    
    """   
    
    # Prepare message info
    header = pack_header(DAS.header)
    left_js = pack_jointstate(DAS.left_joint_state)
    left_eef = pack_pose(DAS.left_eef_state)
    left_J = pack_jacobian(DAS.left_jacobian)
    right_js = pack_jointstate(DAS.right_joint_state)
    right_eef = pack_pose(DAS.right_eef_state)
    right_J = pack_jacobian(DAS.right_jacobian)
    rel_J = pack_jacobian(DAS.rel_jacobian)
    
    # Package message data
    DAS_msg = {"header": header,
               "left_joint_state": left_js,
               "left_jacobian": left_J,
               "left_eef_state": left_eef,
               "right_joint_state": right_js,
               "right_jacobian": right_J,
               "right_eef_state": right_eef,
               "rel_jacobian": rel_J}
               
    return DAS_msg

def pack_jacobian(J):
    """
    
    """
    
    # Prepare message info
    name = J.name
    frame = J.frame
    row0 = J.row0
    row1 = J.row1
    row2 = J.row2
    row3 = J.row3
    row4 = J.row4
    row5 = J.row5
    
    # Package message data
    J_msg = {"name": name,
             "frame": frame,
             "row0": row0,
             "row1": row1,
             "row2": row2,
             "row3": row3,
             "row4": row4,
             "row5": row5}
    
    return J_msg
