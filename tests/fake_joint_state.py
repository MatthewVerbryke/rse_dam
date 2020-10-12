#!/usr/bin/env python


import sys

import rospy
from sensor_msgs.msg import JointState


def fake_joint_state(side):
    """
    
    """
    
    # Initialize Node
    rospy.init_node("fake_joint_state_publisher", anonymous=True)
    
    # Parameters 
    joints = ["{}_shoulder_1_joint".format(side),
              "{}_shoulder_2_joint".format(side),
              "{}_elbow_joint".format(side),
              "{}_wrist_1_joint".format(side),
              "{}_wrist_2_joint".format(side),
              "{}_wrist_3_joint".format(side),
              "{}_gripper_joint".format(side),
              "{}_gripper_prismatic_joint_2".format(side)]

    # ROS publisher
    pub = rospy.Publisher("boxbot/{}_arm/joint_states".format(side),
                          JointState,
                          queue_size=1)
    
    command = JointState()
    command.name = joints
    command.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        command.header.stamp = rospy.Time.now()
        pub.publish(command)
        r.sleep()
        

if __name__ == "__main__":
    
    side = sys.argv[1]
    fake_joint_state(str(side))
