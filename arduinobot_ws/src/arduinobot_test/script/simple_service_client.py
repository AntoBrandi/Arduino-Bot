#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
from arduinobot_test.srv import AddTwoInts


if __name__ == "__main__":
    # When this script is launched, get the args that are passed
    # to the script when is launched. This script requires 2 args from the user
    # that are the two numbers that will be added
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    # The number of args passed to this function is not correct
    else:
        print("%s [x y]"%sys.argv[0])
        sys.exit(1)

    print("Requesting %s+%s"%(x, y))

    # Sum the two numbers passed as arguments to this script 
    # by calling the action server that adds two integers
    rospy.wait_for_service('add_two_ints')
    add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
    response = add_two_ints(x, y)

    # Show the response of the service
    print("Response %s"%response.sum)
