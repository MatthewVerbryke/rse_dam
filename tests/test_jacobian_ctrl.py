#!/usr/bin/env python


import sys

import rospy

from geometry_msgs.msg import Pose, PoseArray
from rse_dam_msgs.msg import HLtoDL, DLtoHL, HLtoRL
from csa_msgs.msg import TimedPoseArray

file_dir = sys.path[0]
sys.path.append(file_dir + "/..")
sys.path.append(file_dir + "/../..")
import deliberative.tactics.trajectories as traj

start_pose = Pose()
start_pose.position.x = 0.209
start_pose.position.y = 0.125
start_pose.position.z = 0.245
start_pose.orientation.x = 0.0
start_pose.orientation.y = -0.707107
start_pose.orientation.z = 0.707107
start_pose.orientation.w = 0.0
    
goal_pose = Pose()
goal_pose.position.x = 0.209
goal_pose.position.y = 0.025
goal_pose.position.z = 0.245
goal_pose.orientation.x = 0.0
goal_pose.orientation.y = -0.707107
goal_pose.orientation.z = 0.707107
goal_pose.orientation.w = 0.0

rel_pose = Pose()
rel_pose.position.x = 0.0
rel_pose.position.y = 0.0
rel_pose.position.z = 0.2
rel_pose.orientation.x = 1.0
rel_pose.orientation.y = 0.0
rel_pose.orientation.z = 0.0
rel_pose.orientation.w = 0.0

speed = 0.01 #m/s
master_traj, timestamps = traj.create_simple_move_trajectory(start_pose, goal_pose, speed, "torso")
slave_traj = traj.create_static_trajectory(rel_pose, timestamps, "left_wrist_2_link")
times = []
for time in timestamps:
    time_float = time["secs"] + 0.000000001*time["nsecs"]
    time_dur = rospy.Duration(time_float)
    times.append(time_dur)
print "time to end:"
print times[-1].to_sec()
print ""
print "number of waypoints"
print len(times)
print ""
print "distance between waypoints"
print abs(goal_pose.position.y - start_pose.position.y)/len(times)
#exit()
trjty = TimedPoseArray()
trjty.poses = master_traj.poses + slave_traj.poses
trjty.time_from_start = times + times

msg = HLtoRL()
msg.new_command = True
msg.ctrl_type = 2
msg.poses = trjty

def pubber():
    pub = rospy.Publisher("HLtoRL", HLtoRL, queue_size=1)
    rospy.init_node("pubber", anonymous=True)
    rate = rospy.Rate(10)
    i=1
    while i < 6:
        pub.publish(msg)
        rate.sleep()
        i += 1
        
if __name__ == "__main__":
    try:
        pubber()
    except rospy.ROSInterruptException:
        pass
