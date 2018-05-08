#!/usr/bin/env python

"""
  Functions for direct packing/unpacking of ROS message types into JSON 
  messages for use with rosbridge. Includes functions for:
  
    -std_msgs/Header
    -geometry_msgs/Pose
    -geometry_msgs/PoseStamped

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
"""


from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs import Header


def pack_header(header):
    """
    Package 'std_msgs/Header' message.
    """
    
    # Get header info
    seq = header.seq
    secs = header.stamp.secs
    nsecs = header.stamp.nsecs
    frame = header.frame_id
    
    # Place into dictionary
    header_msg = {"seq": seq,
                  "stamp": {"secs": secs, "nsecs": nsecs},
                  "frame_id": frame}
                  
    return header_msg
    
def unpack_header(dictmsg):
    """
    Unpackage 'std_msgs/Header' message.
    """
    
    # Get info from message
    header_msg = dictmsg["msg"]
    seq = header_msg["seq"]
    sec = header_msg["stamp"]["secs"]
    nsec = header_msg["stamp"]["nsecs"]
    frame = header_msg["frame_id"]
    
    # Package into ROS message
    header = Header()
    header.seq = seq
    header.stamp.secs = sec
    header.stamp.nsecs = nsec
    header.frame_id = frame
    
    return header
    

def pack_pose(pose):
    """
    Package 'geometry_msgs/Pose' message.
    """
    
    # Get pose info
    x = posearray.poses[i].position.x
    y = posearray.poses[i].position.y
    z = posearray.poses[i].position.z
    qx = posearray.poses[i].orientation.x
    qy = posearray.poses[i].orientation.y
    qz = posearray.poses[i].orientation.z
    qw = posearray.poses[i].orientation.w
    
    # Package into dict
    pose_msg = {"position": {"x": x, "y": y, "z": z},
                 "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}}
    
    return pose_msg
    
def unpack_pose(dictmsg):
    """
    Unpackage 'geometry_msgs/Pose' message.
    """
    
    # Get info from message
    pose_msg = dictmsg["msg"]
    x = msg["position"]["x"]
    y = msg["position"]["y"]
    z = msg["position"]["z"]
    qx = msg["orientation"]["x"]
    qy = msg["orientation"]["y"]
    qz = msg["orientation"]["z"]
    qw = msg["orientation"]["w"]
    
    # Package into ROS msg
    pose = Pose()
    pose.position.x = x 
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    
    return pose
    

def pack_posestamped(posestamped):
    """
    Package 'geometry_msgs/PoseStamped' message.
    """
    
    # Get header and pose messages 
    header_msg = pack_header(posestamped.header)
    pose_msg = pack_pose(posestamped.pose)
    
    # Package into dict
    posestamped_msg = {"header": header_msg,
                       "pose": pose_msg}
                       
    return posestamped
    
def unpack_posestamped(dictmsg):
    """
    Unpackage 'geometry_msgs/PoseStamped' message.
    """
    
    # Get info from message (reusing previous functions)
    posestamped_msg = dictmsg["msg"]
    header_msg = {"msg": posestamped_msg["header"]}
    header = unpack_header(header_msg)
    pose_msg = {"msg": posestamped_msg["pose"]}
    pose =  unpack_pose(pose_msg)
    
    # Package into ROS message
    posestamped = PoseStamped()
    posestamped.header = header
    posestamped.pose = pose
    
    return posestamped


def pack_posearray(posearray):
    """
    
    """
    pass

def unpack_posearray(dictmsg):
    """
    
    """
    pass
    

def pack_diagnosticarray(datalist):
    """
    
    """
    pass
    
def unpack_diagnosticarray(dictmsg):
    """
    
    """
    pass


def pack_robottrajectory(datalist):
    """
    
    """
    pass
    
def unpack_robottrajectory(dictmsg):
    """
    
    """
    pass

