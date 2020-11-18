#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Bool
from moveit_interface import MoveitInterface
from pick_task import PickTask
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

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

# constant list of the joint names for the arm
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3']


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
    task = PickTask()
    if data.data:
        task.execute()


if __name__ == '__main__':
    # Inizialize a ROS node called remote_controller
    rospy.init_node('remote_controller', anonymous=True)

    # Init the interface with moveit plugin
    moveit = MoveitInterface()

    # Register a simple action client that uses the action servers declared in the controller packages
    # for the execution of the arm joint trajectories and the gripper joint
    client_trajectory = actionlib.SimpleActionClient('/arduinobot_arm_controller/trajectory_action', control_msgs.msg.FollowJointTrajectoryAction)
    client_gripper = actionlib.SimpleActionClient('/arduinobot_gripper_controller/gripper_action', control_msgs.msg.GripperCommandAction)
    client_trajectory.wait_for_server()
    client_gripper.wait_for_server()
    

    # register a subscriber on the topics /jarvis_messenger that will listen for String messages
    # and the topic /jarvis_controller that will receive JointState messages
    # and the topic /jarvis_voice that will receive Bool messages
    # when a new message is received, the callback function is triggered and starts its execution
    rospy.Subscriber("jarvis_controller", JointState, jarvis_controller_callback)
    rospy.Subscriber("jarvis_messenger", String, jarvis_messenger_callback)
    rospy.Subscriber("jarvis_voice", Bool, jarvis_voice_callback)

    # Keep the subscriber running
    rospy.spin()