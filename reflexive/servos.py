#!/usr/bin/env python

"""
  Servo objects for use with the ArbotiX reflexive layer. Mostly taken
  from:
  https://github.com/MatthewVerbryke/arbotix_ros/blob/turtlebot2i/arbotix_python/src/arbotix_python/servo_controller.py
 
  Copyright (c) 2011-2013 Vanadium Labs LLC.
  Copyright (c) 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test pretty much everything
"""


from math import radians

import rospy
from diagnostic_msgs.msg import DiagnosticStatus


class DynamixelServo(object):
    """
    An object for storing data and commands for a controlled Dynamixel
    servo.
    """
    
    def __init__(self, name):
        
        # Get name of joint this servo controls
        self.name = named
        namespace = "~joints/" + name + "/"
        
        # Get data on this servo from the ROS parameter server
        self.id = int(rospy.get_param(n+"id"))
        self.ticks = rospy.get_param(n+"ticks", 1024)
        self.range = rospy.get_param(n+"range", 360.0)
        self.max_angle = radians(rospy.get_param(n+"max_angle",self.range/2.0))
        self.min_angle = radians(rospy.get_param(n+"min_angle",-self.range/2.0))
        self.max_speed = radians(rospy.get_param(n+"max_speed",684.0))
        self.invert = rospy.get_param(n+"invert",False)
        
        # Calculate other needed parameters
        self.rad_per_tick = radians(self.range)/self.tick)
        self.neutral = (self.max_angle + self.min_angle)/2
        
        # Initialize variables for this servo
        self.status = "OK"
        self.level = DiagnosticStatus.OK
        self.dirty = False              # newly updated position?
        self.position = 0.0             # current position (radians)
        self.jst_position= 0.0          # position to report as Joint Status
        self.desired = 0.0              # desired position (radians)
        self.last_cmd = 0.0             # last position sent (radians)
        self.velocity = 0.0             # moving speed
        self.enabled = True             # can we take commands?
        self.active = False             # are we under torque control?
        self.last = rospy.Time.now()
        self.reads = 0.0                # number of reads
        self.errors = 0                 # number of failed reads
        self.total_reads = 0.0                  
        self.total_errors = [0.0]
        self.voltage = 0.0
        self.temperature = 0.0
        self.load = 0.0
        self.tolerance = 0.05
        rospy.loginfo("Started Servo %d  %s", self.id, name)

    def interpolate(self, frame):
        """
        Get the new position to move to, in ticks.
        """
        
        if self.enabled and self.active and self.dirty:
            
            # Cap movement
            if abs(self.last_cmd - self.desired) < self.tolerance:
                self.dirty = False
            
            # Compute command, limit velocity
            cmd = self.desired - self.last_cmd
            if cmd > self.max_speed/frame:
                cmd = self.max_speed/frame
            elif cmd < -self.max_speed/frame:
                cmd = -self.max_speed/frame
            
            # Compute angle, apply limits
            ticks = self.angle_to_ticks(self.last_cmd + cmd)
            self.last_cmd = self.ticks_to_angle(ticks)
            self.speed = cmd*frame
            
            return int(ticks)
            
    def jst_position(self, pos):
        """
        Report Joint Status in radians
        """
        
        return self.ticks_to_angle(pos) 

    def set_current_feedback(self, reading):
        """
        Update angle in radians by reading from servo, or by 
        using position passed in from a sync read (in ticks).
        """
        
        # Check validity
        if reading > -1 and reading < self.ticks:
            
            # Get the current angular position
            self.reads += 1
            self.total_reads += 1
            last_angle = self.position
            self.position = self.ticks_to_angle(reading)
            self.jst_position = self.jst_position(reading)
            
            # Update the velocity estimate
            t = rospy.Time.now()
            self.velocity = (self.position - last_angle)/((t - self.last).to_nsec()/1000000000.0)
            self.last = t
            
        # Handle reading errors
        else:
            rospy.logdebug("Invalid read of servo: id " + str(self.id) + ", value " + str(reading))
            self.errors += 1
            self.total_reads += 1
            return
            
        # If inactive
        if not self.active:
            self.last_cmd = self.position
        
    def set_control_output(self, position):
        """
        Set the position that controller is moving to. Returns output 
        value in ticks.
        """
        
        if self.enabled:
            ticks = self.angle_to_ticks(position)
            self.desired = position
            self.dirty = True
            self.active = True
            
            return int(ticks)
            
        return -1

    def get_diagnostics(self):
        """
        Build a diagnostic status message.
        """
        
        msg = DiagnosticStatus()
        msg.name = self.name
        
        # Handle servo overheating
        if self.temperature > 60:
            self.status = "OVERHEATED, SHUTDOWN"
            self.level = DiagnosticStatus.ERROR
        elif self.temperature > 50: 
            self.status = "OVERHEATING"
            self.level = DiagnosticStatus.WARN
        else:
            self.status = "OK"
            self.level = DiagnosticStatus.OK
        
        # Construct the diagnostic message
        msg.level = self.level
        msg.message = self.status
        msg.values.append(KeyValue("Position", str(self.position)))
        msg.values.append(KeyValue("Temperature", str(self.temperature)))
        msg.values.append(KeyValue("Voltage", str(self.voltage)))
        if self.reads + self.errors > 100:
            self.total_errors.append((self.errors*100.0)/(self.reads+self.errors))
            if len(self.total_errors) > 10:
                self.total_errors = self.total_errors[-10:]
            self.reads = 0
            self.errors = 0
        msg.values.append(KeyValue("Reads", str(self.total_reads)))
        msg.values.append(KeyValue("Error Rate", str(sum(self.total_errors)/len(self.total_errors))+"%" ))
        if self.active:
            msg.values.append(KeyValue("Torque", "ON"))
        else:
            msg.values.append(KeyValue("Torque", "OFF"))
            
        return msg

    def angle_to_ticks(self, angle):
        """
        Convert an angle to ticks, applying limits.
        """
        
        ticks = self.neutral + (angle/self.rad_per_tick)
        if self.invert:
            ticks = self.neutral - (angle/self.rad_per_tick)
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
            
        return ticks

    def ticks_to_angle(self, ticks):
        """
        Convert an ticks to angle, applying limits.
        """
        
        angle = (ticks - self.neutral) * self.rad_per_tick
        if self.invert:
            angle = -1.0 * angle
            
        return angle

    def speed_to_ticks(self, rads_per_sec):
        """
        Convert speed in radians per second to ticks, applying limits.
        """
        
        ticks = self.ticks * rads_per_sec / self.max_speed
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
            
        return ticks
