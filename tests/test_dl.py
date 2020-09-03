#!/usr/bin/env python


import rospy

from geometry_msgs.msg import Pose, PoseArray
from rse_dam_msgs.msg import HLtoDL, DLtoHL
import deliberative.trajectories as traj

start_pose = Pose()
start_pose.position.x = 0.24
start_pose.position.y = 0.128
start_pose.position.z = 0.2659
start_pose.orientation.x = 0.0
start_pose.orientation.y = -0.707107
start_pose.orientation.z = 0.707107
start_pose.orientation.w = 0.0
    
goal_pose = Pose()
goal_pose.position.x = 0.24
goal_pose.position.y = 0.2
goal_pose.position.z = 0.2659
goal_pose.orientation.x = 0.0
goal_pose.orientation.y = -0.707107
goal_pose.orientation.z = 0.707107
goal_pose.orientation.w = 0.0

speed = 0.01 #m/s
obj_trajectory, timestamps = traj.create_simple_move_trajectory(start_pose, goal_pose, speed, "origin_point")

msg = DLtoHL()
msg.new_cmd = 1
msg.move_type = 1
msg.poses = obj_trajectory
msg.stamps = str(timestamps)
msg.target_pose = Pose()
msg.command = 1


def pubber():
    pub = rospy.Publisher("deliberative/to_hl", DLtoHL, queue_size=10)
    rospy.init_node("pubber", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
        print("published")
        
if __name__ == "__main__":
    try:
        pubber()
    except rospy.ROSInterruptException:
        pass