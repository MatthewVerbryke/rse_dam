#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

from rse_dam_msgs.msg import HLtoDL, DLtoHL, OptoDL, DLtoOp


header = Header()
header.seq = 0
header.stamp.secs = 1
header.stamp.nsecs = 100000000
header.frame_id = "world"

pose1 = Pose()
pose1.position.x = 0.3
pose1.position.y = 0.2
pose1.position.z = 0.1
pose1.orientation.x = 0.0
pose1.orientation.y = 1.0
pose1.orientation.z = 0.0
pose1.orientation.w = 0.0

pose2 = Pose()
pose2.position.x = 0.4
pose2.position.y = 0.3
pose2.position.z = 0.2
pose2.orientation.x = 0.0
pose2.orientation.y = 0.0
pose2.orientation.z = 1.0
pose2.orientation.w = 0.0

parray = PoseArray()
parray.header = header
parray.poses.append(pose1)
parray.poses.append(pose2)

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

# Trial messages
dltohl = DLtoHL()
dltohl.status = 1
dltohl.move_type = 2
dltohl.poses = parray
dltohl.stamps = "jkldjakjsflds"
dltohl.target_pose = pose1
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
optodl.object_start = pose1
optodl.object_goal = pose2
optodl.left_grasp_pose = pose2
optodl.left_gap = 1.0
optodl.right_grasp_pose = pose1
optodl.right_gap = 2.0
optodl.move_speed = 3.0
optodl.lift_height = 4.0

def pubber():
    pub = rospy.Publisher("/layer_test", OptoDL, queue_size=1)
    rospy.init_node("pubber", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(optodl)
        rate.sleep()
        print("published")
        
if __name__ == "__main__":
    try:
        pubber()
    except rospy.ROSInterruptException:
        pass

