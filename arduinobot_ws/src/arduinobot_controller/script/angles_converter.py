#!/usr/bin/env python
import rospy
import math
from arduinobot_controller.srv import AnglesConverter, AnglesConverterResponse

def convert_radians_to_degrees(req):
    resp = AnglesConverterResponse()
    resp.base = int(((req.base+(math.pi/2))*180)/math.pi)
    resp.shoulder = 180-int(((req.shoulder+(math.pi/2))*180)/math.pi)
    resp.elbow = int(((req.elbow+(math.pi/2))*180)/math.pi)
    resp.gripper = int(((-req.gripper)*180)/(math.pi/2))
    return resp

def convert_degrees_to_radians(req):
    resp = AnglesConverterResponse()
    resp.base = ((math.pi*req.base) - ((math.pi/2)*180))/180
    resp.shoulder = (((180-req.shoulder)*math.pi)-((math.pi/2)*180))/180
    resp.elbow = ((math.pi*req.elbow) - ((math.pi/2)*180))/180
    resp.gripper = -((math.pi/2)*req.gripper)/180
    return resp


if __name__ == "__main__":
    rospy.init_node('angles_converter')

    radians_to_degrees = rospy.Service('radians_to_degrees', AnglesConverter, convert_radians_to_degrees)
    degrees_to_radians = rospy.Service('degrees_to_radians', AnglesConverter, convert_degrees_to_radians)

    rospy.spin()