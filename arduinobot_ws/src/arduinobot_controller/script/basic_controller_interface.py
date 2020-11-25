#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray
from arduinobot_controller.srv import AnglesConverter

"""
  arduinobot - basic_controller_interface
  
  This script implements an interface between ROS Master on the PC/Raspberry side and the
  Arduino Controller with a ROS node that actuates the servo motors

  The aim of the script is to convert sensor_msgs/JointState messages into 
  a std_msgs/UInt16MultiArray message that is lighter and so more suitable for 
  Arduino to be received and used.
  The order of each joint in the array is the following

  - base
  - shoulder
  - elbow
  - gripper

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""
    

def callback(data):
    # This function is called each time a new message is published on the topic /joint_states
    # this message is a sensor_msg/JointStatmessage type
    # get the data and publish those on the connected arduino as joint angles
    # The message that was published on the topic /joint_states is then passed to this function as input
    radians_to_degrees = rospy.ServiceProxy('radians_to_degrees', AnglesConverter)
    angles_deg = radians_to_degrees(data.position[0],data.position[1],data.position[2],data.position[3])

    # compose the array message
    msg = [int(angles_deg.base), int(angles_deg.shoulder), int(angles_deg.elbow), int(angles_deg.gripper)]
    print "Angles %s ", msg
    # publish the array message to the defined topic
    pub.publish(data=msg)
    
if __name__ == '__main__':
    # Inizialize a ROS node called basic_controller_interface
    rospy.init_node('basic_controller_interface', anonymous=True)

    # register a publisher on the topic /servo_actuate that will publish UInt16MultiArray messages
    pub = rospy.Publisher('servo_actuate', UInt16MultiArray, queue_size=10)

    # register a subscriber on the topic /joint_states that will listen for JointState messages
    # when a new message is received, the callback function is triggered and starts its execution
    rospy.Subscriber("/joint_states", JointState, callback)

    rospy.wait_for_service('angles_converter')


    # keep this ROS node up and running
    rospy.spin()