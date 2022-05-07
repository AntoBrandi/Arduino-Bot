#!/usr/bin/env python3
import rospy
from arduinobot_test.srv import AddTwoInts
import sys


if __name__ == "__main__":
    # When this script is launched, get the args that are passed
    # to the script when is launched. This script requires 2 args from the user
    # that are the two numbers that will be added
    if len(sys.argv) == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    # The number of args passed to this function is not correct
    else:
       print('Requested two arguments')
       sys.exit(1)

    print('Requesting ', a, b)

    # Sum the two numbers passed as arguments to this script 
    # by calling the action server that adds two integers
    rospy.wait_for_service('add_two_ints')
    add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
    response = add_two_ints(a, b)

    # Show the response of the service
    print('Service Response ', response)