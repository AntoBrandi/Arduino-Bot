#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64MultiArray, Float64

"""
  arduinobot - fake_controller_interface
  This script implements a broker between the gazebo joint position controller taht actuates the simulated robot 
  via Float64 messages on the topics /arduinobot_sim/joint_**joint_number**_position_controller/command
  and the FollowJointTrajectoryAction and GripperCommandAction trajectory controllers that 
  implements an Action Server in order to actuate the arm and the gripper with a give trajectory

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""

# constant list of the joint names 
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']


def arm_actuate_cb(data):
    # Callback function that is called each time a new message is published on the topic /arduinobot_arm_controller/arduino_sim/arm_actuate
    # the received message is passed as input to this function
    # It uses the received message to actuate the arm joints of the simulated robot
    pub_joint_1.publish(data.data[0])
    pub_joint_2.publish(data.data[1])
    pub_joint_3.publish(data.data[2])

def gripper_actuate_cb(data):
    # Callback function that is called each time a new message is published on the topic arduinobot_gripper_controller/arduino_sim/gripper_actuate
    # the received message is passed as input to this function
    # It uses the received message to actuate the gripper joints of the simulated robot
    pub_joint_4.publish(data.data)
    pub_joint_5.publish(-data.data)


if __name__ == '__main__':
    # Inizialize a ROS node called fake_controller_interface
    rospy.init_node('fake_controller_interface', anonymous=True)

    # register the publishers for the position controller topics that are publishing Float64 messages
    # which indicates the angle in radians that a given joint will be actuated
    pub_joint_1 = rospy.Publisher('/arduinobot_sim/joint_1_position_controller/command', Float64, queue_size=10)
    pub_joint_2 = rospy.Publisher('/arduinobot_sim/joint_2_position_controller/command', Float64, queue_size=10)
    pub_joint_3 = rospy.Publisher('/arduinobot_sim/joint_3_position_controller/command', Float64, queue_size=10)
    pub_joint_4 = rospy.Publisher('/arduinobot_sim/joint_4_position_controller/command', Float64, queue_size=10)
    pub_joint_5 = rospy.Publisher('/arduinobot_sim/joint_5_position_controller/command', Float64, queue_size=10)

    # register a subscriber on the topic /arduinobot_arm_controller/arduino_sim/arm_actuate and on tthe topic 
    # arduinobot_gripper_controller/arduino_sim/gripper_actuate that will listen for Float64MultiArray messages indicating the 
    # desidered joint angles for the arm joint and will listen for Float64 messages indicating the desidered joint angle
    # for the gripper joint
    # when a new message is received, the callback function is triggered and starts its execution
    rospy.Subscriber("arduinobot_arm_controller/arduino_sim/arm_actuate", Float64MultiArray, arm_actuate_cb)
    rospy.Subscriber("arduinobot_gripper_controller/arduino_sim/gripper_actuate", Float64, gripper_actuate_cb)
    
    # keep this ROS node up and running
    rospy.spin()