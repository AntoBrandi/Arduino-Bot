#!/usr/bin/env python
import rospy
from std_msgs.msg import String

"""
  arduinobot - simple_remote_listener

  This script implements a simple subscriber on the topic /jarvis_messenger and prints
  all the received messages to the screen.
  It is a sample node whose aim is to show how it is possible to receive ROS messages
  that are published by external entities
  

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""


def jarvis_messenger_callback(data):
    # function that gets called every time a new message is published on the topic /jarvis_messenger
    # it prints the received String message in the console log
    rospy.loginfo('Remote message received %s', data.data)


if __name__ == '__main__':
    # Inizialize a ROS node called simple_remote_listener
    rospy.init_node('simple_remote_listener', anonymous=True)

    # register a subscriber on the topic /jarvis_messenger that will listen for String messages
    # when a new message is received, the callback function is triggered and starts its execution
    rospy.Subscriber("jarvis_messenger", String, jarvis_messenger_callback)

    # Keep the subscriber running
    rospy.spin()