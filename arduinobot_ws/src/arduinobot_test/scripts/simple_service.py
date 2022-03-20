#!/usr/bin/env python3
from arduinobot_test.srv import AddTwoInts, AddTwoIntsResponse
import rospy


def add_two_ints(req):
    # Function that is called each time the service receives a request
    # The request message is passed as input to this function
    rospy.loginfo('Ready to sum %d and %d', req.a, req.b)
    return AddTwoIntsResponse(req.a + req.b)

if __name__ == "__main__":
    # Inizialize a ROS node called simple_service
    rospy.init_node('simple_service')

    # Initialize a service named add_two_ints that uses the AddTwoInts
    # for the Request/Response communication interface
    # Define the function that is called each time the service receives a request
    service = rospy.Service('add_two_ints', AddTwoInts, add_two_ints)

    rospy.loginfo("Ready to add two ints.")

    # keeps the node up and running
    rospy.spin()