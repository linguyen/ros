#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(x, y, w):
    # Initialize the ROS node
    rospy.init_node('move_turtlebot3_to_goal', anonymous=True)

    # Create a SimpleActionClient for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait until the action server is up and running
    client.wait_for_server()

    # Define the goal position
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w

    # Send the goal to move_base
    client.send_goal(goal)

    # Wait for the result
    wait = client.wait_for_result()
    
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.logerr("Doing!!!")
        return client.get_result()

if __name__ == '__main__':
    try:
        # Set your goal coordinates here
        x_goal = 2.0
        y_goal = 2.0
        w_orientation = 1.0

        result = move_to_goal(x_goal, y_goal, w_orientation)

        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
