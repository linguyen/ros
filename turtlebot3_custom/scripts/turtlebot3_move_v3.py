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
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class Turtlebot3Controller:
    def __init__(self):
        rospy.init_node('turtlebot3_controller', anonymous=True)
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.rate = rospy.Rate(10)
        self.move_cmd = Twist()
        self.start_position = Pose()
        self.current_position = Pose()
        self.min_distance = float('inf')

        self.set_start_position()  # Set the initial position as the start position

    def scan_callback(self, msg):
        self.min_distance = min(msg.ranges)

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose

    def set_start_position(self):
        # Assuming the initial position when the script starts is the start position
        rospy.sleep(1)  # Wait for odometry data to be available
        self.start_position = self.current_position
        #.loginfo(f"Start position set to: ({self.start_position.position.x}, {self.start_position.position.y})")

    def move_to(self, linear_speed, angular_speed):
        self.move_cmd.linear.x = linear_speed
        self.move_cmd.angular.z = angular_speed
        self.pub.publish(self.move_cmd)

    def distance_to_start(self):
        dx = self.start_position.position.x - self.current_position.position.x
        dy = self.start_position.position.y - self.current_position.position.y
        return math.sqrt(dx**2 + dy**2)

    def return_to_base(self):
        while not rospy.is_shutdown():
            if self.min_distance < 0.5:  # If an obstacle is detected within 0.5 meters
                rospy.loginfo("Obstacle detected! Turning to avoid.")
                self.move_to(0, 0)
                rospy.sleep(1)
                self.move_to(0, 1.57)  # Turn 90 degrees
                rospy.sleep(1)  # Allow time for the turn to complete
                self.move_to(0, 0)
            else:
                distance = self.distance_to_start()
                if distance < 0.1:  # Close to start position
                    self.move_to(0, 0)
                    rospy.loginfo("Returned to base position.")
                    break
                else:
                    #rospy.loginfo(f"Distance to start: {distance:.2f} meters")
                    self.move_to(0.2, 0)  # Move forward

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = Turtlebot3Controller()
        rospy.loginfo("Returning to base position...")
        controller.return_to_base()
    except rospy.ROSInterruptException:
        pass