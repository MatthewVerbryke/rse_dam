#!/usr/bin/env python


import ast
import rospy
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import tf

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
#from deliberative.deliberative import *
from deliberative.adapter import *
from deliberative.trajectories import *
#from habitual.test_moveit import *
#from habitual.reachability import *


REF_FRAME = "origin_point"

def offset_test():
    
    start_pose = Pose()
    start_pose.position.x = 0.0
    start_pose.position.y = 0.0
    start_pose.position.z = 0.0
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 1.0
    start_frame = pose_to_frame_matrix(start_pose)
    
    goal_pose = Pose()
    goal_pose.position.x = 0.0
    goal_pose.position.y = 0.027
    goal_pose.position.z = 0.2659
    goal_pose.orientation.x = -0.707099
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 0.707099  
     
    offset = compute_eef_offset(start_pose, goal_pose)
    
    test = np.dot(start_frame, offset)
    test_pose = frame_matrix_to_pose(test)
    print test_pose
    
def tf_lookup_test():
    
    deliberative = DeliberativeModule()
    
    tf = deliberative.lookup_frame_transform("base_link", "left_wrist_2_link")
    
    print tf
    
def ik_test():
    
    interface = RSEMoveItInterface()
    
    #quat = tf.transformations.quaternion_from_euler(-90,0,-90)
    
    test_goal = PoseStamped()
    test_goal.header.frame_id = REF_FRAME
    test_goal.header.stamp = rospy.Time.now()
    test_goal.pose.position.x = 0.19095
    test_goal.pose.position.y = 0.222533
    test_goal.pose.position.z = 0.265902
    test_goal.pose.orientation.x = 0.00331337
    test_goal.pose.orientation.y = -0.707099
    test_goal.pose.orientation.z = 0.707099
    test_goal.pose.orientation.w = -0.00331337
    
    traj = interface.plan_to_pose_goal(test_goal)

def ik_serv_test():
    
    interface = RSEMoveItInterface()
    
    test_goal = PoseStamped()
    test_goal.header.frame_id = REF_FRAME
    test_goal.header.stamp = rospy.Time.now()
    test_goal.pose.position.x = 0.19095
    test_goal.pose.position.y = 0.222533
    test_goal.pose.position.z = 0.265902
    test_goal.pose.orientation.x = 0.00331337
    test_goal.pose.orientation.y = -0.707099
    test_goal.pose.orientation.z = 0.707099
    test_goal.pose.orientation.w = -0.00331337
    
    joint_state = interface.ik_solve(test_goal)
    print joint_state
    
    ############### Solution Returned ###############
    #solution: 
    #  joint_state: 
    #    header: 
    #      seq: 0
    #      stamp: 
    #        secs: 0
    #        nsecs:         0
    #      frame_id: "/base_footprint"
    #    name: [joint_1, joint_2, joint_3, joint_4, joint_5, gripper_joint, gripper_prismatic_joint_2]
    #    position: [0.861632959449934, 0.23833962245154486, -0.3390386369514542, 0.10069901449990581, 2.4230576261253653, 0.0165, 0.0165]
    #    velocity: []
    #    effort: []
    #  multi_dof_joint_state: 
    #    header: 
    #      seq: 0
    #      stamp: 
    #        secs: 0
    #        nsecs:         0
    #      frame_id: "/base_footprint"
    #    joint_names: []
    #    transforms: []
    #    twist: []
    #    wrench: []
    #  attached_collision_objects: []
    #  is_diff: False
    #error_code: 
    #  val: 1

def fk_serv_test():
    
    interface = RSEMoveItInterface()
    
    test_goal = JointState()
    test_goal.header.frame_id = REF_FRAME
    test_goal.header.stamp = rospy.Time.now()
    test_goal.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    test_goal.position = [0.861632959449934, 0.23833962245154486, -0.3390386369514542, 0.10069901449990581, 2.4230576261253653]
    print test_goal
    pose_stamped = interface.fk_solve(test_goal, "wrist_2_link")
    print pose_stamped
    
def traj_test():
    
    start_pose = Pose()
    start_pose.position.x = 0.0
    start_pose.position.y = -0.0
    start_pose.position.z = 0.0
    start_pose.orientation.x = 0.707107
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 0.707107
    
    goal_pose = Pose()
    goal_pose.position.x = 0.0
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.0
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = -0.707107
    goal_pose.orientation.z = 0.707107
    goal_pose.orientation.w = 0.0
    
    speed = 0.05 #rad/s
    lift_height = 0.1
    
    trajectory, timestamps = create_in_place_rotation_trajectory(start_pose, goal_pose, speed, REF_FRAME)
    #trajectory, timestamps = create_simple_move_trajectory(start_pose, goal_pose, speed, REF_FRAME)    
    #trajectory = create_pick_and_place_trajectory(start_pose, goal_pose, lift_height, speed)

    timestamp_array = ast.literal_eval(str(timestamps))

    print_traj = False
    
    if print_traj:
        for i in range(0,len([trajectory])):
            leng = len(trajectory.poses)
            xs = [None]*leng
            ys = [None]*leng
            zs = [None]*leng
            for j in range(0,leng):
                xs[j] = trajectory.poses[j].pose.position.x
                ys[j] = trajectory.poses[j].pose.position.y    
                zs[j] = trajectory.poses[j].pose.position.z
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(xs, ys, zs, c="b")
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')
            plt.show()
    print trajectory
    print timestamps
    return trajectory 
    
def frame_to_pose_test():
    
    adapter = RobotTrajectoryAdapter()
    
    test_goal = PoseStamped()
    test_goal.header.frame_id = REF_FRAME
    test_goal.header.stamp = rospy.Time.now()
    test_goal.pose.position.x = 0.19095
    test_goal.pose.position.y = 0.222533
    test_goal.pose.position.z = 0.265902
    test_goal.pose.orientation.x = 0.00331337
    test_goal.pose.orientation.y = -0.707099
    test_goal.pose.orientation.z = 0.707099
    test_goal.pose.orientation.w = -0.00331337
    
    test_frame = adapter.pose_to_frame_matrix(test_goal)
    test_pose = adapter.frame_matrix_to_pose(test_frame)

def arm_pose_test():
    
    adapter = RobotTrajectoryAdapter()
    
    obj_trajectory = traj_test()
    
    object_pose = Pose()
    object_pose.position.x = 0.2
    object_pose.position.y = -0.1
    object_pose.position.z = 0.0
    object_pose.orientation.x = 1.0
    object_pose.orientation.y = 0.0
    object_pose.orientation.z = 0.0
    object_pose.orientation.w = 0.0
    
    left_pose = Pose()
    left_pose.position.x = 0.2
    left_pose.position.y = -0.05
    left_pose.position.z = 0.0
    left_pose.orientation.x = 1.0
    left_pose.orientation.y = 0.0
    left_pose.orientation.z = 0.0
    left_pose.orientation.w = 0.0
    
    right_pose = Pose()
    right_pose.position.x = 0.2
    right_pose.position.y = -0.15
    right_pose.position.z = 0.0
    right_pose.orientation.x = 1.0
    right_pose.orientation.y = 0.0
    right_pose.orientation.z = 0.0
    right_pose.orientation.w = 0.0
    
    left_offset = adapter.compute_eef_offset(object_pose, right_pose)
    #right_offset = adapter.compute_eef_offset(object_pose, left_pose)
    
    for trajectory in obj_trajectory:
        left_traj = adapter.adapt_arm_poses(trajectory, left_offset)
    
def ik_traj_test():
    
    #adapter = RobotTrajectoryAdapter()
    
    interface = RSEMoveItInterface()
    
    start_pose = Pose()
    start_pose.position.x = 0.2
    start_pose.position.y = 0.1
    start_pose.position.z = 0.2659
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = -0.707107
    start_pose.orientation.z = 0.707107
    start_pose.orientation.w = 0.0
    
    goal_pose = Pose()
    goal_pose.position.x = 0.2
    goal_pose.position.y = 0.2
    goal_pose.position.z = 0.2659
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = -0.707107
    goal_pose.orientation.z = 0.707107
    goal_pose.orientation.w = 0.0
    
    interface.arm.set_goal_orientation_tolerance(0.01)
    interface.plan_to_pose_goal(start_pose)
    #print(interface.trajectory)
    
    interface.execute_trajectory()#; exit()
    start_real = interface.get_current_eef_pose()
    start_pose_real = start_real.pose
    speed = 0.01 #m/s
    obj_trajectory, timestamps = create_simple_move_trajectory(start_pose_real, goal_pose, speed, "base_link")
    
    interface.get_trajectory_from_eef_poses(obj_trajectory, str(timestamps))
    #print(interface.trajectory)#; exit()
    interface.execute_trajectory()
    
    interface.cleanup()
    
def ik_traj_test2():
    
    #adapter = RobotTrajectoryAdapter()
    
    interface = MoveItHabitualModule()
    
    interface.arm.set_goal_orientation_tolerance(0.01)
    start_pose = Pose()
    start_pose.position.x = 0.24
    start_pose.position.y = 0.128
    start_pose.position.z = 0.2659
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = -0.707107
    goal_pose.orientation.z = 0.707107
    goal_pose.orientation.w = 0.0
    
    goal_pose = Pose()

    interface.plan_to_pose_goal(start_pose)
    interface.execute_trajectory()
    exit()


#offset_test()
#tf_lookup_test()
#ik_test()
#ik_serv_test()
#fk_serv_test()
traj_test()
#frame_to_pose_test()
#arm_pose_test()
#ik_traj_test()
#ik_traj_test2()

