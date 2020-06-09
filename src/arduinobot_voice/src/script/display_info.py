#!/usr/bin/env python
import sys
import copy
import rospy
from std_msgs.msg import String
from goal_publisher import GoalPublisher


# Instantiate the goal publisher class
gp = GoalPublisher()


if __name__ == '__main__':
    gp.getRobotInfo()