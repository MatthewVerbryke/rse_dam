#!/usr/bin/env python

"""
  Functions for direct packing/unpacking of ROS message types into JSON 
  messages for use with rosbridge. Includes functions for:
  
    -std_msgs/Header
    -geometry_msgs/Pose
    -geometry_msgs/PoseStamped
    -geometry_msgs/PoseArray
    -moveit_msgs/RobotTrajectory

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
"""


from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Header


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
    header.frame_id = str(frame)
    
    return header
    

def pack_pose(pose):
    """
    Package 'geometry_msgs/Pose' message.
    """
    
    # Get pose info
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w
    
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
    x = pose_msg["position"]["x"]
    y = pose_msg["position"]["y"]
    z = pose_msg["position"]["z"]
    qx = pose_msg["orientation"]["x"]
    qy = pose_msg["orientation"]["y"]
    qz = pose_msg["orientation"]["z"]
    qw = pose_msg["orientation"]["w"]
    
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
                
    return posestamped_msg
    
def unpack_posestamped(dictmsg):
    """
    Unpackage 'geometry_msgs/PoseStamped' message.
    """
    
    # Get info from message
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
    Package 'geometry_msgs/PoseArray' message.
    
    TODO: TEST
    """

    # Get header messages
    header_msg = pack_header(posearray.header)
    
    # Setup loop
    pose_num = len(posearray.poses)
    pose_list_msg = [None]*pose_num

    # Get and store a message for each pose
    for i in range(0,pose_num):
        pose_msg_i = pack_pose(posearray.poses[i])
        pose_list_msg[i] = pose_msg_i
        
    # Package into a dict
    posearray_msg = {"header": header_msg,
                     "poses": pose_list_msg}
                     
    return posearray_msg

def unpack_posearray(dictmsg):
    """
    Unpackage 'geometry_msgs/PoseStamped' message.
    
    TODO: FINISH?
    """
    
    # Get info from message
    posearray_msg = dictmsg["msg"]
    header_msg = {"msg": posearray_msg["header"]}
    header = unpack_header(header_msg)
    
    pass


def pack_diagnosticarray(datalist):
    """
    TODO
    """
    pass
    
def unpack_diagnosticarray(dictmsg):
    """
    TODO
    """
    pass


def pack_robottrajectory(robottrajectory):
    """
    Package 'geometry_msgs/PoseStamped' message.
    """
    
    # Get header message
    jt_header_msg = pack_header(robottrajectory.joint_trajectory.header)
    
    # Get joint name message
    joint_list = robottrajectory.joint_trajectory.joint_names
    
    # Setup loop
    jtpoints_num = len(robottrajectory.joint_trajectory.points)
    jtpoints_list = [None]*jtpoints_num
    
    # Get messages for all JointTrajectoryPoints
    for i in range(0,jtpoints_num):
        
        # Get info from current point
        current_point = robottrajectory.joint_trajectory.points[i]
        positions = current_point.positions
        velocities = current_point.velocities
        accelerations = current_point.accelerations
        effort = current_point.effort
        time_secs = current_point.time_from_start.secs
        time_nsecs = current_point.time_from_start.nsecs
        
        # Package into dict
        jtpoint_msg = {"positions": positions,
                       "velocities": velocities,
                       "accelerations": accelerations,
                       "effort": effort,
                       "time_from_start": {"secs": time_secs, "nsecs": time_nsecs}}
        
        # Put into list
        jtpoints_list[i] = jtpoint_msg
        
    # Assemble the message
    rt_msg = {"joint_trajectory": {"header": jt_header_msg,
                                   "joint_names": joint_list,
                                   "points": jtpoints_list},
              "multi_dof_joint_trajectory": {}} # <-- okay being empty?
              
    return rt_msg

def unpack_robottrajectory(dictmsg):
    """
    TODO
    """
    pass

