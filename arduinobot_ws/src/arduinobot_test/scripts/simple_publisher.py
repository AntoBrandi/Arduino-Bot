#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    # Inizialize a ROS node called talker
    rospy.init_node('simple_publisher_py', anonymous=True)

    # register a publisher on the topic /chatter that will publish String messages
    pub = rospy.Publisher('chatter', String, queue_size=10)
        
    # Define the frequency for publishing the messages
    # The rate is expressed in Hertz
    rate = rospy.Rate(10) # 10hz
    counter = 0

    # Keep going publishing messages until the ROS communication is alive
    while not rospy.is_shutdown():
        hello_msg = "hello world %d" % counter
        pub.publish(hello_msg)
        # wait the desired rate before publishing the next message
        rate.sleep()
        counter += 1