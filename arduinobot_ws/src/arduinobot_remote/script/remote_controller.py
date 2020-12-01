#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import UInt16
from moveit_interface import MoveitInterface
from task import Task
import random

"""
  arduinobot - remote_controller

  This script implements an interface between the robot and the external world.
  It is in charge of receive messages from the external world, eventually validate them,
  and publish those on the correct topic of the robot where those will be used by other modules

  The open interface is listening on topics:

  - jarvis_messenger : Simple String messages are published. It may be useful 
                       for sending command to the robot via messages coming from different sources
                       (es. Telegram actuated robot)

  - jarvis_controller : JointState messages are published. It receives desidered poses of each joint 
                        of the robot and executes them

  - jarvis_voice : Bool messages are published. It receives a trigger for the execution of a pre recorded task
  

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""


def jarvis_controller_callback(data):
    # function that gets called every time a new message is published on the topic /jarvis_voice
    # triggers the execution of a pre-recorded task
    moveit.reach_goal(data)

def jarvis_messenger_callback(data):
    # function that gets called every time a new message is published on the topic /jarvis_messenger
    rospy.loginfo('Remote message received: %s', data.data)  

def jarvis_voice_callback(data):
    # function that gets called every time a new message is published on the topic /jarvis_voice
    # it start the execution of a previously recorded task
    task = Task()
    # pick
    if data.data == 0:
        task.add_position([1.5, 0.0, -0.4, -1.0, 1.0])
        task.add_position([1.5, -0.6, -0.4, -1.0, 1.0])
        task.add_position([1.5, -0.6, -0.4, 0.0, 0.0])
        task.add_position([1.5, -0.6, 0.8, 0.0, 0.0])
        task.add_position([1.5, 0.0, 0.8, 0.0, 0.0])
        task.add_position([-1.14, -0.6, -0.07, 0.0, 0.0])
        task.set_speed(0.5)
        task.set_acceleration(0.1)
        task.execute()
    # sleep
    elif data.data == 1:
        task.add_position([-1.57,0.0,-1.0,0.0, 0.0])
        task.set_speed(0.7)
        task.set_acceleration(0.1)
        task.execute()
    # wake up
    elif data.data == 2:
        task.add_position([0.0,0.0,0.0,-0.7, 0.7])
        task.set_speed(0.7)
        task.set_acceleration(0.1)
        task.execute()
    # dance
    elif data.data == 3:
        i = 0
        while i<10:
            rand_base = round(random.uniform(-1.57,1.57),2)
            rand_shoulder = round(random.uniform(-1.57,1.57),2)
            rand_elbow = round(random.uniform(-1.57,1.57),2)
            rand_gripper = round(random.uniform(-1.57,0.0),2)
            task.add_position([rand_base, rand_shoulder, rand_elbow, rand_gripper, -rand_gripper])
            i +=1
        task.set_speed(1)
        task.set_acceleration(1)
        task.execute()
    # invalid
    else:
        rospy.loginfo('cannot execute the task: %s', data.data)        


if __name__ == '__main__':
    # Inizialize a ROS node called remote_controller
    rospy.init_node('remote_controller', anonymous=True)

    # Init the interface with moveit plugin
    moveit = MoveitInterface()

    # register a subscriber on the topics /jarvis_messenger that will listen for String messages
    # and the topic /jarvis_controller that will receive JointState messages
    # and the topic /jarvis_voice that will receive Bool messages
    # when a new message is received, the callback function is triggered and starts its execution
    rospy.Subscriber("jarvis_controller", JointState, jarvis_controller_callback)
    rospy.Subscriber("jarvis_messenger", String, jarvis_messenger_callback)
    rospy.Subscriber("jarvis_voice", UInt16, jarvis_voice_callback)

    # Keep the subscriber running
    rospy.spin()