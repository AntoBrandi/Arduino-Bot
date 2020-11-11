#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# function that gets called every time a new message is published on the topic /jarvis
def jarvis_messenger_callback(data):
    rospy.loginfo('Remote message received %s', data.data)


if __name__ == '__main__':
    rospy.init_node('remote_listener', anonymous=True)

    # Subscribe to the topic in which the voice assistant is writing
    rospy.Subscriber("jarvis_messenger", String, jarvis_messenger_callback)

    # Keep the subscriber running
    rospy.spin()