#!/usr/bin/env python3
import rospy


def timerCallback(event=None):
    # Function that will be called at each expiration of the timer
    rospy.loginfo('Called timerCallback Function')


if __name__ == '__main__':
    try:
        # Inizialize a ROS node called timer node
        rospy.init_node('timer_node', anonymous=True)

        # Set the duration of the timer
        timer_duration = rospy.Duration(1)
        # Create an instance of the timer with its duration and the function that
        # will be called at each time duration
        rospy.Timer(timer_duration, timerCallback)

        # Keep the node up and running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass