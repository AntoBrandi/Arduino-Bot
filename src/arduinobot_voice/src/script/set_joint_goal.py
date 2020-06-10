#!/usr/bin/env python
import sys
import copy
import rospy
from std_msgs.msg import String
from goal_publisher import GoalPublisher


# Instantiate the goal publisher class
gp = GoalPublisher()

# function that gets called every time a new message is published on the topic /jarvis
def jarvis_callback(data):
    rospy.loginfo('Robot triggered')

    # Move the robot
    # Set a target goal for the robot in the cartesian space
    gp.setRobotJoint()



if __name__ == '__main__':
    # Subscribe to the topic in which the voice assistant is writing
    rospy.Subscriber("jarvis", String, jarvis_callback)

    # Keep the subscriber running
    rospy.spin()    