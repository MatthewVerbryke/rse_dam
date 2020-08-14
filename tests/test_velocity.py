#!/usr/bin/env python


from std_msgs.msg import Header
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class VelocityCheckTest:
    
    def __init__(self):
        
        self.joint_max_velocity = [0.785, 1.571, 1.571]
        
        self.header = Header()
        self.header.seq = 0
        self.header.stamp.secs = 1
        self.header.stamp.nsecs = 100000000
        self.header.frame_id = "world"
        
        self.jt_point1 = JointTrajectoryPoint()
        self.jt_point1.positions = [0.0, 0.0, 0.0]
        self.jt_point1.time_from_start.secs = 0
        self.jt_point1.time_from_start.nsecs = 000000000
        
        self.jt_point2 = JointTrajectoryPoint()
        self.jt_point2.positions = [0.1, 0.2, 0.2]
        self.jt_point2.time_from_start.secs = 1
        self.jt_point2.time_from_start.nsecs = 100000000
        
        self.jt_point3 = JointTrajectoryPoint()
        self.jt_point3.positions = [0.1, 1.4, 56.3]
        self.jt_point3.time_from_start.secs = 2
        self.jt_point3.time_from_start.nsecs = 200000000
        
        self.left_traj = RobotTrajectory()
        self.left_traj.joint_trajectory.header = self.header
        self.left_traj.joint_trajectory.joint_names = ["joint_1", "joint_2", "joint_3"]
        self.left_traj.joint_trajectory.points.append(self.jt_point1)
        self.left_traj.joint_trajectory.points.append(self.jt_point2)
        self.left_traj.joint_trajectory.points.append(self.jt_point3)
        
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
        
    def check_trajectory(self, side):
        """
        Check to make sure the trajectory returned is safe and is 
        consistant with constraints. WIP
        
        TODO: TEST
        """
        
        # Setup
        if (side=="left"):
            traj_points = self.left_traj.joint_trajectory.points
        elif (side=="right"):
            traj_points = self.right_traj.joint_trajectory.points
        joint_num = len(self.joint_max_velocity)
        interval_num = len(traj_points) - 1 
        
        # Check that average speed for each servo are below limitations
        #   NOTE: avg_speed = (angle2 - angle1)/time
        for i in range(0,interval_num):
            for joint in range(0,joint_num):
                prev_angle = traj_points[i].positions[joint]
                next_angle = traj_points[i+1].positions[joint]
                prev_time = self.get_float_time(traj_points[i].time_from_start)
                next_time = self.get_float_time(traj_points[i+1].time_from_start)
                avg_speed = (next_angle - prev_angle)/(next_time - prev_time); print avg_speed
                if (abs(avg_speed)<self.joint_max_velocity[joint]):
                    pass # Should be okay
                else:
                    self.fail_info = "Joint {0} on the {1} exceeds maximum velocity limit in current trajectory".format(joint, side)
                    return False

        return True
        
if __name__ == "__main__":
    v = VelocityCheckTest()
    ret = v.check_trajectory("left")
    if not ret:
        print v.fail_info
