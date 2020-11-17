#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64MultiArray, Float64

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']


def arm_actuate_cb(data):
    pub_joint_1.publish(data.data[0])
    pub_joint_2.publish(data.data[1])
    pub_joint_3.publish(data.data[2])

def gripper_actuate_cb(data):
    pub_joint_4.publish(data.data)


if __name__ == '__main__':

    rospy.init_node('fake_controller_interface', anonymous=True)

    pub_joint_1 = rospy.Publisher('/arduinobot_sim/joint_1_position_controller/command', Float64, queue_size=10)
    pub_joint_2 = rospy.Publisher('/arduinobot_sim/joint_2_position_controller/command', Float64, queue_size=10)
    pub_joint_3 = rospy.Publisher('/arduinobot_sim/joint_3_position_controller/command', Float64, queue_size=10)
    pub_joint_4 = rospy.Publisher('/arduinobot_sim/joint_4_position_controller/command', Float64, queue_size=10)

    rospy.Subscriber("arduinobot_arm_controller/arduino_sim/arm_actuate", Float64MultiArray, arm_actuate_cb)
    rospy.Subscriber("arduinobot_gripper_controller/arduino_sim/gripper_actuate", Float64, gripper_actuate_cb)
    
    rospy.spin()