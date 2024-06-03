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
from turtlesim.msg import Pose as TurtlePose
from math import sqrt, atan2
from random import uniform

class TurtlesController:
    def __init__(self):
        self.pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        self.sub1 = rospy.Subscriber('/turtle1/pose', TurtlePose, self.turtle1_callback)
        self.sub1 = rospy.Subscriber('/turtle2/pose', TurtlePose, self.turtle2_callback)
        rospy.init_node('move_turtles', anonymous=True)
        self.rate = rospy.Rate(10)
        self.turtle1_pose = TurtlePose()
        self.turtle2_pose = TurtlePose()
        self.turtle1_pose.x = 10
        self.turtle1_pose.y = 10
        self.turtle1_pose.theta = 0.0

    def turtle1_callback(self, msg):
        self.turtle1_pose = msg

    def turtle2_callback(self, msg):
        self.turtle2_pose = msg

    def distance_between_turtles(self):
        dx = self.turtle1_pose.x - self.turtle2_pose.x
        dy = self.turtle1_pose.y - self.turtle2_pose.y
        return sqrt(dx**2 + dy**2)

    def move_turtles(self):
        while not rospy.is_shutdown():
            distance = self.distance_between_turtles()
            rospy.loginfo(distance)
            if distance < 1.0:
                stop_cmd = Twist()
                self.pub1.publish(stop_cmd)
                self.pub2.publish(stop_cmd)
                rospy.loginfo("Turtles have met. Stopping.")
                break

            else: 
                move_cmd1 = Twist()
                move_cmd1.linear.x = 1.0

                move_cmd2 = Twist()
                move_cmd2.linear.x = 1.0

                self.pub1.publish(move_cmd1)
                self.pub2.publish(move_cmd2)
                
                rospy.loginfo("Turtles are going forward.")

                self.rate.sleep()


if __name__ == '__main__':
    try:
        turtles_controler = TurtlesController()
        turtles_controler.move_turtles()
    except rospy.ROSInterruptException:
        pass
