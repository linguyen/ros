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

#!/usr/bin/env python

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

    def scan_callback(self, scan_data):
        self.twist.linear.x = 0
        # Publish the velocity command
        self.cmd_vel_publisher.publish(self.twist)

if __name__ == '__main__':
    rospy.init_node('turtlebot3_controller')
    
    # Create controller instances for each TurtleBot3
    turtlebot3_burger_controller = TurtleBot3Controller('turtlebot1')
    turtlebot3_burger_2_controller = TurtleBot3Controller('turtlebot2')
    
    rospy.spin()
