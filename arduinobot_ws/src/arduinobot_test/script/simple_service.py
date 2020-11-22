#!/usr/bin/env python
from __future__ import print_function
from arduinobot_test.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

if __name__ == "__main__":
    rospy.init_node('simple_service')
    service = rospy.Service('add_two_ints', AddTwoInts, add_two_ints)
    rospy.spin()