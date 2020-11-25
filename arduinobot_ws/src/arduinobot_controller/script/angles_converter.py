#!/usr/bin/env python
import rospy
import math
from arduinobot_controller.srv import AnglesConverter, AnglesConverterResponse

"""
  arduinobot - angles_converter
  This script implements two services on the topics
    - radians_to_degrees
    - degrees_to_radians

  Both of them receives a request with the format:
    float64 base
    float64 shoulder
    float64 elbow 
    float64 gripper
  and sends a response in the same format to the client

  The first service (radians_to_degrees) receives the angles in radians and convert
  those in degrees according to the boundaries defined inthe URDF file

  The second service (degrees_to_radians) receives the angles in degrees and convert
  those in radians according to the boundaries defined inthe URDF file

  This conversion is needed for the control of the real robot in order to convert the radians angle of each joint
  as used in ROS in degrees angles as used in Arduino for the actuation of the Servo motors
  

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""

def convert_radians_to_degrees(req):
    # Function that is called every time the service radians_to_degrees is called
    # It receives the Request message as input with the angles in radians
    # and returns the Result message as output with the angles in degrees
    res = AnglesConverterResponse()
    res.base = int(((req.base+(math.pi/2))*180)/math.pi)
    res.shoulder = 180-int(((req.shoulder+(math.pi/2))*180)/math.pi)
    res.elbow = int(((req.elbow+(math.pi/2))*180)/math.pi)
    res.gripper = int(((-req.gripper)*180)/(math.pi/2))
    return res

def convert_degrees_to_radians(req):
    # Function that is called every time the service radians_to_degrees is called
    # It receives the Request message as input with the angles in degrees
    # and returns the Result message as output with the angles in radians
    res = AnglesConverterResponse()
    res.base = ((math.pi*req.base) - ((math.pi/2)*180))/180
    res.shoulder = (((180-req.shoulder)*math.pi)-((math.pi/2)*180))/180
    res.elbow = ((math.pi*req.elbow) - ((math.pi/2)*180))/180
    res.gripper = -((math.pi/2)*req.gripper)/180
    return res


if __name__ == "__main__":
    # Inizialize a ROS node called angles_converter
    rospy.init_node('angles_converter')

    # Inizialize two services for the angle conversions 
    radians_to_degrees = rospy.Service('radians_to_degrees', AnglesConverter, convert_radians_to_degrees)
    degrees_to_radians = rospy.Service('degrees_to_radians', AnglesConverter, convert_degrees_to_radians)

    # keeps the node up and running
    rospy.spin()