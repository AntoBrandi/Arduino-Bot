#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

"""
  arduinobot - basic_fake_controller_interface
  This script implements an interface between ROS Master on the PC/Raspberry side and the
  gazebo fake controllers that actuates the robot

  The aim of the script is to convert sensor_msgs/JointState messages into 
  a std_msgs/Float64 messages for each joint of the robot
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
    anglesRad = data.position

    # publish the joint states to the controllers that
    # actuates each joint of the simulated robot in gazebo
    pub_joint_1.publish(anglesRad[0])
    pub_joint_2.publish(anglesRad[1])
    pub_joint_3.publish(anglesRad[2])
    pub_joint_4.publish(anglesRad[3])
    pub_joint_5.publish(anglesRad[4])
    
if __name__ == '__main__':
    # Inizialize a ROS node called basic_fake_controller_interface
    rospy.init_node('basic_fake_controller_interface', anonymous=True)

    # register the publishers for the position controller topics that are publishing Float64 messages
    # which indicates the angle in radians that a given joint will be actuated
    pub_joint_1 = rospy.Publisher('/arduinobot_sim/joint_1_position_controller/command', Float64, queue_size=10)
    pub_joint_2 = rospy.Publisher('/arduinobot_sim/joint_2_position_controller/command', Float64, queue_size=10)
    pub_joint_3 = rospy.Publisher('/arduinobot_sim/joint_3_position_controller/command', Float64, queue_size=10)
    pub_joint_4 = rospy.Publisher('/arduinobot_sim/joint_4_position_controller/command', Float64, queue_size=10)
    pub_joint_5 = rospy.Publisher('/arduinobot_sim/joint_5_position_controller/command', Float64, queue_size=10)

    # register a subscriber on the topic /joint_states that will listen for JointState messages
    # when a new message is received, the callback function is triggered and starts its execution
    rospy.Subscriber("/joint_states", JointState, callback)

    # keep this ROS node up and running
    rospy.spin()