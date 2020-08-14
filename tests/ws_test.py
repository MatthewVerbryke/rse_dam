#!/usr/bin/env python

import sys
import time
import communication.packing as compack
import communication.rse_packing as rsepack
from rse_dam_msgs.msg import HLtoDL, DLtoHL, OptoDL, DLtoOp

file_dir = sys.path[0]
sys.path.append(file_dir + '/..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState


if __name__ == "__main__":    

    pub_or_sub = raw_input("publishing(p) or subscribing(s)? ")
    connection = "ws://192.168.0.51:9090/" #left-to-right
    #connection = "ws://192.168.0.50:9090/" #right-to-left

    if (pub_or_sub=="s"):
        # sub test
        
        ws = rC.RosMsg('ws4py', connection, 'sub', '/string_test', 'std_msgs/String', compack.unpack_string)
        
        out = None
        while (out is None):
            out = ws.receive()
            if out is None:
                print("No data recieved yet")
            else:
                print("Data recieved!")
                print(out)
            time.sleep(0.5)
                
    elif (pub_or_sub=="p"):
        # pub test
        
        header = Header()
        header.seq = 0
        header.stamp.secs = 1
        header.stamp.nsecs = 100000000
        header.frame_id = "world"
        
        test_timestamp_array = str([{"secs": 0, "nsecs": 10000000},
                                    {"secs": 0, "nsecs": 20000000}, 
                                    {"secs": 0, "nsecs": 50000000}])
        
        goal_pose = Pose()
        goal_pose.position.x = 0.2
        goal_pose.position.y = 0.1
        goal_pose.position.z = 0.0
        goal_pose.orientation.x = 1.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = 0.0
        goal_pose.orientation.w = 0.0
        
        goal_pose1 = Pose()
        goal_pose1.position.x = 0.3
        goal_pose1.position.y = 0.2
        goal_pose1.position.z = 0.1
        goal_pose1.orientation.x = 0.0
        goal_pose1.orientation.y = 1.0
        goal_pose1.orientation.z = 0.0
        goal_pose1.orientation.w = 0.0
        
        goal_pose2 = Pose()
        goal_pose2.position.x = 0.4
        goal_pose2.position.y = 0.3
        goal_pose2.position.z = 0.2
        goal_pose2.orientation.x = 0.0
        goal_pose2.orientation.y = 0.0
        goal_pose2.orientation.z = 1.0
        goal_pose2.orientation.w = 0.0
        
        test_parray = PoseArray()
        test_parray.header = header
        test_parray.poses.append(goal_pose)
        test_parray.poses.append(goal_pose1)
        test_parray.poses.append(goal_pose2)
        
        jt_point1 = JointTrajectoryPoint()
        jt_point1.positions = [0.0, 1.0, 2.0]
        jt_point1.time_from_start.secs = 0
        jt_point1.time_from_start.nsecs = 00000000
        
        jt_point2 = JointTrajectoryPoint()
        jt_point2.positions = [1.0, 2.0, 3.0]
        jt_point2.time_from_start.secs = 0
        jt_point2.time_from_start.nsecs = 10000000
        
        jt_point3 = JointTrajectoryPoint()
        jt_point3.positions = [2.0, 3.0, 4.0]
        jt_point3.time_from_start.secs = 0
        jt_point3.time_from_start.nsecs = 20000000
        
        test_traj = RobotTrajectory()
        test_traj.joint_trajectory.header = header
        test_traj.joint_trajectory.joint_names = ["joint_1", "joint_2", "joint_3"]
        test_traj.joint_trajectory.points.append(jt_point1)
        test_traj.joint_trajectory.points.append(jt_point2)
        test_traj.joint_trajectory.points.append(jt_point3)
        
        test_js = JointState()
        test_js.header = header
        test_js.name = ["joint_1", "joint_2", "joint_3"]
        test_js.position = [1.0, 2.0, 3.0]
        test_js.velocity = [2.0, 3.0, 4.0]
        test_js.effort = [0.0, 0.0, 0.0]
        
        test_float = 0.0
        
        dltohl = DLtoHL()
        dltohl.status = 1
        dltohl.move_type = 2
        dltohl.poses = test_parray
        dltohl.stamps = "jkldjakjsflds"
        dltohl.target_pose = goal_pose1
        dltohl.command = 3

        hltodl = HLtoDL()
        hltodl.status = 1
        hltodl.state = 2
        hltodl.trajectory = test_traj

        dltoop = DLtoOp()
        dltoop.dl_status = 1
        dltoop.right_status = 2
        dltoop.left_status = 3

        optodl = OptoDL()
        optodl.move_type = 1
        optodl.object_start = goal_pose1
        optodl.object_goal = goal_pose2
        optodl.left_grasp_pose = goal_pose2
        optodl.left_gap = 1.0
        optodl.right_grasp_pose = goal_pose1
        optodl.right_gap = 2.0
        optodl.move_speed = 3.0
        optodl.lift_height = 4.0
        
        ws = rC.RosMsg('ws4py', connection, 'pub', '/test', 'rse_dam_msgs/HLtoDL', rsepack.pack_HL_to_DL)
        while (1):
            ws.send(hltodl)
            time.sleep(0.01)
            #print("msg sent!")

        ws.close()
        
    
