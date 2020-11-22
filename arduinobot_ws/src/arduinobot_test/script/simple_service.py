#!/usr/bin/env python
from __future__ import print_function
from arduinobot_test.srv import AddTwoInts,AddTwoIntsResponse
import rospy


def add_two_ints(req):
    # Function that is called each time the service receives a request
    # The request message is passed as input to this function
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

if __name__ == "__main__":
    # Inizialize a ROS node called simple_service
    rospy.init_node('simple_service')

    # Initialize a service named add_two_ints that uses the AddTwoInts
    # for the Request/Response communication interface
    # Define the function that is called each time the service receives a request
    service = rospy.Service('add_two_ints', AddTwoInts, add_two_ints)

    # keeps the node up and running
    rospy.spin()