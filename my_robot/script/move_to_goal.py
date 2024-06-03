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
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class MoveToGoal:
    def __init__(self):
        rospy.init_node('move_to_goal', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_odom)
        self.pose = None
        self.rate = rospy.Rate(10)

    def update_odom(self, data):
        self.pose = data.pose.pose
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

    def euclidean_distance(self, goal_pose):
        return math.sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) +
                         pow((goal_pose.position.y - self.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.position.y - self.pose.position.y,
                          goal_pose.position.x - self.pose.position.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.yaw)

    def move_to_goal(self, goal_x, goal_y):
        goal_pose = Odometry().pose.pose
        goal_pose.position.x = goal_x
        goal_pose.position.y = goal_y

        distance_tolerance = 0.1
        vel_msg = Twist()

        while not rospy.is_shutdown():
            if self.pose is None:
                continue

            if self.euclidean_distance(goal_pose) >= distance_tolerance:
                vel_msg.linear.x = self.linear_vel(goal_pose)
                vel_msg.angular.z = self.angular_vel(goal_pose)

                self.velocity_publisher.publish(vel_msg)
            else:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                break

            self.rate.sleep()

        rospy.spin()

if __name__ == '__main__':
    try:
        x_goal = float(input("Set your x goal: "))
        y_goal = float(input("Set your y goal: "))
        mover = MoveToGoal()
        mover.move_to_goal(x_goal, y_goal)
    except rospy.ROSInterruptException:
        pass
