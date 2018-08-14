#!/usr/bin/env python

"""
  Functions to parse .bag files for data specific to rse_dam scenarios
  
  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.
"""


import math
import sys

from geometry_msgs.msg import Pose
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import tf.transformations as transf


class RSESimBagParser(object):
    """
    Parse bag file data and prepare for figure printing for sim data.
    """
    
    def __init__(self):
        
        # Open the bag file
        bag_name = sys.argv[1]
        self.bag = rosbag.Bag(str(bag_name))
        
        # Get what data needs to be graphed
        self.apply_tf = True
        self.graph_clock = False
        self.graph_eef_pose = True
        self.graph_contacts = False
        self.graph_block = True
        
        # Initialize holding lists
        self.state_time = []
        self.system_time = []
        self.left_contact_time = []
        self.right_contact_time = []
        self.contact_time = []
        self.clock_time = []
        self.left_eef_state = []
        self.right_eef_state = []
        self.left_force = []
        self.left_torque = []
        self.right_force = []
        self.right_torque = []
        self.block_force = []
        self.block_torque = []
        
        # Run
        self.main()

    def parse_clock (self):
        """
        Parse the .bag file for the gazebo clock time.
        """
        
        # Parse the 'clock' topic to get the relation between system time and clock time
        for topic, msg, t in self.bag.read_messages(topics=['/clock']):
            
            # Convert stamp into float time
            time = self.get_float_time(t)
            self.system_time.append(time)
            
            # Parse the clock time 
            cl_time = self.get_float_time(msg.clock)
            self.clock_time.append(cl_time)
        
    def parse_eef_states(self):
        """
        Parse the .bag file for end effector state information data.
        """
        # Parse the '/link_states' topic to get the desired link states out
        for topic, msg, t in self.bag.read_messages(topics=['/gazebo/link_states']):

            # Convert stamp into float time
            time = self.get_float_time(t)
            self.state_time.append(time) 
            
            # Parse eef_states
            left_index = msg.name.index("boxbot::left_wrist_2_link")
            self.left_eef_state.append(msg.pose[left_index])
            right_index = msg.name.index("boxbot::right_wrist_2_link")
            self.right_eef_state.append(msg.pose[right_index])
            
    def parse_contact_states(self, topic_name):
        """
        Parse the .bag file for contact state data
        """
        
        time_out = []
        forces_x = []
        forces_y = []
        forces_z = []
        torques_x = []
        torques_y = []
        torques_z = []
        
        # Parse right contact states topic to get contact force data out
        for topic, msg, t in self.bag.read_messages(topics=[topic_name]):
            
            # Convert stamp into float time
            r_time = self.get_float_time(msg.header.stamp)
            time_out.append(r_time)
            
            # Parse the message for force info
            if (msg.states==[]):
                forces_x.append(0.0)
                forces_y.append(0.0)
                forces_z.append(0.0)
                torques_x.append(0.0)
                torques_y.append(0.0)
                torques_z.append(0.0)
            else:
                force_x = 0.0
                force_y = 0.0
                force_z = 0.0
                torque_x = 0.0
                torque_y = 0.0
                torque_z = 0.0
                for contact in msg.states:
                    force_x = force_x + contact.total_wrench.force.x
                    force_y = force_y + contact.total_wrench.force.y
                    force_z = force_z + contact.total_wrench.force.z
                    torque_x = torque_x + contact.total_wrench.torque.x
                    torque_y = torque_y + contact.total_wrench.torque.y
                    torque_z = torque_z + contact.total_wrench.torque.z
                    
                forces_x.append(force_x)
                forces_y.append(force_y)
                forces_z.append(force_z)
                torques_x.append(torque_x)
                torques_y.append(torque_y)
                torques_z.append(torque_z)
            
        force_out = [forces_x, forces_y, forces_z]
        torques_out = [torques_x, torques_y, torques_z]
        
        return time_out, force_out, torques_out
        
    def parse_trajectory(self, topic_name):
        
        # Parse the trajectory
        for topic, msg, t in self.bag.read_messages(topics=[topic_name]):
            if (msg.trajectory.joint_trajectory.points==[]):
                pass
            else:
                print msg.trajectory
                break
    
    def get_float_time(self, stamp):
        """
        Convert a stamp style representation of time (sec, nsec) into a floating-
        point representation.
        """
        
        # Convert nsec from nsecs (int) to seconds (float)
        float_nsecs = float(stamp.nsecs)*0.000000001
        
        # Add the two components together
        float_secs = float(stamp.secs) + float_nsecs

        return float_secs
        
    def plot_poses(self):
        """
        Plot the poses of the end-effectors
        """
        
        t = []
        lx = []; rx = []
        ly = []; ry = []
        lz = []; rz = []
        
        # Get times
        for time in self.state_time:
            t.append(time)
        
        # Apply tf for test blocks
        if self.apply_tf:
            
            # Get left values
            for pose in self.left_eef_state:
                trans, rot = self.apply_transformation(pose)
                lx.append(trans[0]*100)
                ly.append(trans[1]*100)
                lz.append(trans[2]*100)
                
            # Get right values
            for pose in self.right_eef_state:
                trans, rot = self.apply_transformation(pose)
                rx.append(trans[0]*100)
                ry.append(trans[1]*100)
                rz.append(trans[2]*100)    
        else:
            # Get left values   
            for pose in self.left_eef_state:
                lx.append(pose.position.x*100)
                ly.append(pose.position.y*100)
                lz.append(pose.position.z*100)
                
            # Get left values 
            for pose in self.right_eef_state:
                rx.append(pose.position.x*100)
                ry.append(pose.position.y*100)
                rz.append(pose.position.z*100)

        # Plot the results
        plt.plot(t, lx, c='r', ls='-', label='x')
        plt.plot(t, ly, c='g', ls='-', label='y')
        plt.plot(t, lz, c='b', ls='-', label='z')
        plt.xlabel("Time, s")
        plt.ylabel("Position, cm")
        #plt.yticks(np.arange(-30, 31, 10))
        plt.legend()
        plt.show()
        
        plt.plot(t, rx, c='r', ls='-', label='x')
        plt.plot(t, ry, c='g', ls='-', label='y')
        plt.plot(t, rz, c='b', ls='-', label='z')
        plt.xlabel("Time, s")
        plt.ylabel("Position, cm")
        #plt.yticks(np.arange(-30, 31, 10))
        plt.legend()
        plt.show()
        
    def plot_offsets(self):
        """
        Plot the relative offsets of the end-effectors
        """
        
        t = []
        zdiff = []
        rolldiff = []
        rolldiff2 = []
        x_offset = []
        y_offset = []
        z_offset = []
        position_offset = []
        quat_diff_x = []
        quat_diff_y = []
        quat_diff_z = []
        quat_diff_w = []
        
        if self.apply_tf:
            left_trans, left_rot = self.apply_transformation(self.left_eef_state[0])
            right_trans, right_rot = self.apply_transformation(self.right_eef_state[0])
            start_diff_x = (left_trans[0] - right_trans[0])*100
            start_diff_y = (left_trans[1] - right_trans[1])*100
            start_diff_z = (left_trans[2] - right_trans[2])*100
            start_diff_tot = math.sqrt(start_diff_x**2 + start_diff_y**2 + start_diff_z**2)
            start_left_quat = [left_rot[0], left_rot[1], left_rot[2], left_rot[3]]
            start_right_quat = [right_rot[0], right_rot[1], right_rot[2], right_rot[3]]
        else:
            start_diff_x = (self.left_eef_state[0].position.x - self.right_eef_state[0].position.x)*100
            start_diff_y = (self.left_eef_state[0].position.y - self.right_eef_state[0].position.y)*100
            start_diff_z = (self.left_eef_state[0].position.z - self.right_eef_state[0].position.z)*100
            start_diff_tot = math.sqrt(start_diff_x**2 + start_diff_y**2 + start_diff_z**2)
            start_left_quat = [self.left_eef_state[0].orientation.x, self.left_eef_state[0].orientation.y, self.left_eef_state[0].orientation.z, self.left_eef_state[0].orientation.w]
            start_right_quat = [self.right_eef_state[0].orientation.x, self.right_eef_state[0].orientation.y, self.right_eef_state[0].orientation.z, self.right_eef_state[0].orientation.w]
        
        # Get start info
        start_left_matrix = transf.quaternion_matrix(start_left_quat)
        start_right_matrix = transf.quaternion_matrix(start_right_quat)
        start_left_rot = start_left_matrix[0:3,0:3]
        start_right_rot = start_right_matrix[0:3,0:3]
        start_left_x = np.dot(start_left_rot, [1, 0, 0])
        start_left_y = np.dot(start_left_rot, [0, 1, 0])
        start_left_z = np.dot(start_left_rot, [0, 0, 1])
        start_right_x = np.dot(start_right_rot, [1, 0, 0])
        start_right_y = np.dot(start_right_rot, [0, 1, 0])
        start_right_z = np.dot(start_right_rot, [0, 0, 1])
        start_x_angle = math.acos(np.dot(start_left_x, start_right_x)/(np.linalg.norm(start_left_x)*np.linalg.norm(start_right_x)))
        start_y_angle = math.acos(np.dot(start_left_y, start_right_y)/(np.linalg.norm(start_left_y)*np.linalg.norm(start_right_y)))
        start_z_angle = math.acos(np.dot(start_left_z, start_right_z)/(np.linalg.norm(start_left_z)*np.linalg.norm(start_right_z)))
        
        # Get times
        for time in self.state_time:
            t.append(time)
            
        # Determine offsets
        for i in range(0,len(self.left_eef_state)):
            
            if self.apply_tf:
                left_trans, left_rot = self.apply_transformation(self.left_eef_state[i])
                right_trans, right_rot = self.apply_transformation(self.right_eef_state[i])
                l_pose = self.quat_to_pose(left_trans, left_rot)
                r_pose = self.quat_to_pose(right_trans, right_rot)
            
            else:
                l_pose = self.left_eef_state[i]
                r_pose = self.right_eef_state[i]
                
            # Position
            x_offset.append((l_pose.position.x - r_pose.position.x)*100 - start_diff_x)
            y_offset.append((l_pose.position.y - r_pose.position.y)*100 - start_diff_y)
            z_offset.append((l_pose.position.z - r_pose.position.z)*100 - start_diff_z)
            position_offset.append((math.sqrt((l_pose.position.x - r_pose.position.x)**2 +
                                             (l_pose.position.y - r_pose.position.y)**2 +
                                             (l_pose.position.z - r_pose.position.z)**2))*100 -
                                             start_diff_tot)
            
            # Orientation
            left_quat = [l_pose.orientation.x, l_pose.orientation.y, l_pose.orientation.z, l_pose.orientation.w]
            right_quat = [r_pose.orientation.x, r_pose.orientation.y, r_pose.orientation.z, r_pose.orientation.w]
            quat_diff = transf.quaternion_multiply(left_quat, transf.quaternion_inverse(right_quat))
            quat_diff_x.append(quat_diff[0])
            quat_diff_y.append(quat_diff[1])
            quat_diff_z.append(quat_diff[2])
            quat_diff_w.append(quat_diff[3])

            left_matrix = transf.quaternion_matrix(left_quat)
            right_matrix = transf.quaternion_matrix(right_quat)
            left_rot = left_matrix[0:3,0:3]
            right_rot = right_matrix[0:3,0:3]
            left_x = np.dot(left_rot, [1, 0, 0])
            left_y = np.dot(left_rot, [0, 1, 0])
            left_z = np.dot(left_rot, [0, 0, 1])
            right_x = np.dot(right_rot, [1, 0, 0])
            right_y = np.dot(right_rot, [0, 1, 0])
            right_z = np.dot(right_rot, [0, 0, 1])
            
            rolldiff.append(math.acos(np.dot(left_x, right_x)/(np.linalg.norm(left_x)*np.linalg.norm(right_x))) - start_x_angle)
            rolldiff2.append(math.acos(np.dot(left_y, right_y)/(np.linalg.norm(left_y)*np.linalg.norm(right_y))) - start_y_angle)
            zdiff.append(math.acos(np.dot(left_z, right_z)/(np.linalg.norm(left_z)*np.linalg.norm(right_z))) - start_z_angle)
        
        plt.plot(t, x_offset, c='r', ls='-', label='x')
        plt.plot(t, y_offset, c='g', ls='-', label='y')
        plt.plot(t, z_offset, c='b', ls='-', label='z')
        plt.plot(t, position_offset, c='k', ls=':', label='total')
        plt.xlabel("Time, s")
        plt.ylabel("Error, cm")
        plt.yticks(np.arange(-0.1, 0.11, 0.01))
        plt.legend()
        plt.show()
        
        plt.plot(t, rolldiff, c='r', ls='-', label = 'x-axes error')
        plt.plot(t, rolldiff2, c='g', ls='-', label = 'y-axes error')
        plt.plot(t, zdiff, c='b', ls='-', label = 'z-axes error')
        plt.xlabel("Time, s")
        plt.ylabel("Error, rad")
        plt.yticks(np.arange(-0.01, 0.011, 0.001))
        plt.legend()
        plt.show()
        
    def quat_to_pose(self, trans, rot):
        """
        Convert a transfromation matrix into a pose.
        """
            
        # Create pose from results
        pose_out = Pose()
        pose_out.position.x = trans[0]
        pose_out.position.y = trans[1]
        pose_out.position.z = trans[2]
        pose_out.orientation.x = rot[0]
        pose_out.orientation.y = rot[1]
        pose_out.orientation.z = rot[2]
        pose_out.orientation.w = rot[3]
            
        return pose_out
        
    def apply_transformation(self, pose):
        """
        Apply the static transform to the test blocks to get their frame center.
        """
        
        # Poses
        tf_position = [0.0, 0.0, 0.1264]
        tf_orientation = [0.0, 0.0, 0.0, 1.0]
        pose_position = [pose.position.x, pose.position.y, pose.position.z]
        pose_orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        
        # Get them in matrix format
        tf_trans = transf.translation_matrix(tf_position)
        tf_rot = transf.quaternion_matrix(tf_orientation)
        tf_matrix = transf.concatenate_matrices(tf_trans, tf_rot)
        pose_trans = transf.translation_matrix(pose_position)
        pose_rot = transf.quaternion_matrix(pose_orientation)
        pose_matrix = transf.concatenate_matrices(pose_trans, pose_rot)
        
        # Apply the tf
        block_matrix = np.dot(pose_matrix, tf_matrix)
        
        # Get out new values
        trans = transf.translation_from_matrix(block_matrix)
        rot = transf.quaternion_from_matrix(block_matrix)
        
        return trans, rot

    def plot_block_forces(self, time_data, force_data, torque_data):
        """
        Plot forces and torques on the test block.
        """

        plt.plot(time_data, force_data[0], c='r', ls='-', label='x')
        plt.plot(time_data, force_data[1], c='g', ls='-', label='y')
        plt.plot(time_data, force_data[2], c='b', ls='-', label='z')
        plt.xlabel("Time, s")
        plt.ylabel("Force, N")
        plt.legend()
        plt.show()

        plt.plot(time_data, torque_data[0], c='r', ls='-', label='x')
        plt.plot(time_data, torque_data[1], c='g', ls='-', label='y')
        plt.plot(time_data, torque_data[2], c='b', ls='-', label='z')
        plt.xlabel("Time, sec")
        plt.ylabel("Torque, Nm^2/s^2")
        plt.legend()
        plt.show()
        
    def main(self):
        """
        Main execution loop
        """
        
        # Parse topic data
        self.parse_clock()
        self.parse_eef_states()
        #self.parse_trajectory("/right_habitual/to_dl")
        self.left_contact_time, self.left_force, self.left_torque = self.parse_contact_states('/left_test_block_contact_state')
        self.right_contact_time, self.right_force, self.right_torque = self.parse_contact_states('/right_test_block_contact_state')
        self.contact_time, self.block_forces, self.block_torques = self.parse_contact_states('/block_contact_state')
        
        # See if clock time and recorded time match up (they normally do)
        if self.graph_clock:
            self.plot_graphs(self.system_time, self.clock_time)
        
        # Plot the end-effector pose histories
        if self.graph_eef_pose:
            self.plot_poses()
            self.plot_offsets()
        
        # Plot the loose block forces and position
        if self.graph_block:
            self.plot_block_forces(self.left_contact_time, self.left_force, self.left_torque)
            self.plot_block_forces(self.right_contact_time, self.right_force, self.right_torque)
            #self.plot_block_forces(self.contact_time, self.block_forces, self.block_torques)
            
        self.bag.close()
    
    def close_bag(self):
        """
        Close the bag file.
        """
        self.bag.close()


if __name__ == "__main__":
    RSESimBagParser()
