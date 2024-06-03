#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TurtleBot3Controller:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.namespace = '/' + robot_name
        self.scan_topic = self.namespace + '/scan'
        self.cmd_vel_topic = self.namespace + '/cmd_vel'
        
        self.scan_subscriber = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        self.cmd_vel_publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        
        self.twist = Twist()
        self.stop_distance = 0.5  # meters
        self.avoid_distance = 1.0  # meters for avoiding another robot
        self.state = 'MOVING_FORWARD'
        self.turning_start_time = None
        self.turn_duration = 2.0  # seconds to turn
        self.faraway = 10

    def scan_callback(self, scan_data):
        # Process the laser scan data to find the minimum distance in front of the robot
        min_distance = min(scan_data.ranges)
        
        print("RANGES By SCAN: (" + str(scan_data.ranges) + ")")
        print("STATUS:" + self.state + "(" + str(min_distance) + ")") 

        # Check the current state and decide on actions
        if self.state == 'MOVING_FORWARD':
            if min_distance < self.avoid_distance:
                self.state = 'TURNING'
                self.turning_start_time = rospy.Time.now()
            elif min_distance < self.stop_distance:
                self.state = 'STOPPED'
            else:
                self.twist.linear.x = 0.2  # Move forward
                self.twist.angular.z = 0.0

        elif self.state == 'STOPPED':
            if min_distance >= self.stop_distance:
                self.state = 'MOVING_FORWARD'
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        elif self.state == 'TURNING':
            if rospy.Time.now() - self.turning_start_time > rospy.Duration(self.turn_duration):
                self.state = 'MOVING_FORWARD'
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.5  # Turn to avoid collision
        elif self.state == 'END':
            self.twist.linear.x = 0.0  # Move forward
            self.twist.angular.z = 0.0
            
        # Publish the velocity command
        self.cmd_vel_publisher.publish(self.twist)

if __name__ == '__main__':
    rospy.init_node('turtlebot3_controller')
    
    # Create controller instances for each TurtleBot3
    turtlebot3_burger_controller = TurtleBot3Controller('turtlebot1')
    turtlebot3_burger_2_controller = TurtleBot3Controller('turtlebot2')
    
    rospy.spin()

