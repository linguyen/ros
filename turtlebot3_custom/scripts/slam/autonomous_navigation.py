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
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():
    # Create a simple action client
    client = actionlib.SimpleActionClient('move_base_custom', MoveBaseAction)
    
    # Wait for the action server to be available
    client.wait_for_server()

    # Define goal positions (replace with actual coordinates)
    goals = [
        [1.0, 1.0, 0.0],
        [2.0, 2.0, 0.0],
        [3.0, 3.0, 0.0]
    ]

    for goal in goals:
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = "map"
        goal_pose.target_pose.header.stamp = rospy.Time.now()

        goal_pose.target_pose.pose.position.x = goal[0]
        goal_pose.target_pose.pose.position.y = goal[1]
        goal_pose.target_pose.pose.orientation.w = goal[2]

        # Send the goal to the action server
        client.send_goal(goal_pose)

        # Wait for the result
        client.wait_for_result()

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
        else:
            rospy.loginfo("Failed to reach the goal...")

if __name__ == '__main__':
    try:
        rospy.init_node('autonomous_navigation', anonymous=True)
        movebase_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")