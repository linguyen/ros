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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Initialize global variable to store the minimum distance from obstacles
min_distance = float('inf')  # Initial large value  # minimum distance threshold to detect obstacles in meters

def scan_callback(msg):
    global min_distance
    # Get the minimum distance from the LiDAR scan data
    min_distance = min(msg.ranges)  # capping range to avoid erroneous readings


def move_robot():
    count_turn = 0

    # Initialize the ROS node
    rospy.init_node('turtlebot3_move', anonymous=True)

    # Create a publisher which can "talk" to the robot and tell it to move
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscribe to the /scan topic to get obstacle data
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Set the rate at which to publish messages
    rate = rospy.Rate(10)  # 10 Hz

    # Define the Twist message to send velocity commands
    move_cmd = Twist()

    while not rospy.is_shutdown():
        if min_distance < 0.3:  # If an obstacle is detected within 5 meters
            rospy.loginfo("Obstacle detected")
            # Stop the robot
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            rospy.loginfo("STOP executed")
            rospy.sleep(1)  # Small delay before turning

            # Turn 90 degrees to avoid the obstacle
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.5  # 90 degrees in radians
            pub.publish(move_cmd)
            count_turn = count_turn + 1
            rospy.loginfo("TURN 90 executed: " + str(count_turn))
            rospy.sleep(1)  # Allow time for the turn to complete

            if count_turn == 3:
                # Turn 90 degrees to avoid the obstacle
                move_cmd.linear.x = -0.1
                move_cmd.angular.z = 0.0  # 90 degrees in radians
                pub.publish(move_cmd)
                count_turn = 0
                rospy.loginfo("REVESE executed")
                rospy.sleep(2)  # Allow time for the turn to complete

                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 1.57  # 90 degrees in radians
                pub.publish(move_cmd)
                rospy.sleep(1)  # Allow time for the turn to complete


            move_cmd.linear.x = 0.1
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            rospy.loginfo("GO AHEAD executed")
            rospy.sleep(1)
        else:
            move_cmd.linear.x = 0.2  # Move forward with 0.2 m/s
            move_cmd.angular.z = 0.0  # No rotation
            pub.publish(move_cmd)

        # Publish the command to the robot
        pub.publish(move_cmd)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
