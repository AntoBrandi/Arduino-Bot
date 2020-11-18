#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray

"""
  arduinobot - controller_interface
  This script implements an interface between ROS Master on the PC/Raspberry side and the
  Arduino Controller with a ROS node that actuates the servo motors

  The aim of the script is to convert std_msgs/UInt16MultiArray messages received from Arduino into 
  a sensor_msgs/JointState message that is the one required in ROS for the presentation
  of the joint status related data and that can be used in other ROS components
  The order of each joint in the received message is the following

  - base
  - shoulder
  - elbow
  - gripper

  The Arduino is publishing std_msgs/UInt16MultiArray messages containing the current status of each joint
  in the previous order instead of publishing directly a sensor_msgs/JointState message because is lighter

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""

# constant list of the joint names 
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']


def convert_angles(angles_deg):
    # Function that converts servo motors angles from degrees to radians 
    # according to the angle convention and boundaries expressed in the robot description
    angles_rad = []
    angles_rad.append(((math.pi*angles_deg[0]) - ((math.pi/2)*180))/180)
    angles_rad.append((((180-angles_deg[1])*math.pi)-((math.pi/2)*180))/180)
    angles_rad.append(((math.pi*angles_deg[2]) - ((math.pi/2)*180))/180)
    angles_rad.append(-((math.pi/2)*angles_deg[3])/180)
    return angles_rad


def joint_states_cb(data):
    # Callback function that is called each time a new message is published on the topic /arduinobot/joint_states
    # the received message is passed as input to this function
    # It filles out a new JointState messages using the data contained in the received UInt16MultiArray message
    # and then publishes the composed JointState message on the topic /arduinobot/joint_states
    status = JointState()
    status.header.stamp = rospy.Time.now()
    status.name = JOINT_NAMES
    status.position = convert_angles(data.data)
    pub_status.publish(status)


if __name__ == '__main__':
    # Inizialize a ROS node called controller_interface
    rospy.init_node('controller_interface', anonymous=True)

    # register a publisher on the topic /arduinobot/joint_states that will publish JointState messages
    pub_status = rospy.Publisher('arduinobot/joint_states', JointState, queue_size=10)

    # register a subscriber on the topic /arduino/joint_states that will listen for UInt16MultiArray messages
    # when a new message is received, the callback function is triggered and starts its execution
    rospy.Subscriber("arduino/joint_states", UInt16MultiArray, joint_states_cb)
    
    # keep this ROS node up and running
    rospy.spin()