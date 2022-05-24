#! /usr/bin/env python3
import rospy
import actionlib
from arduinobot_remote.msg import ArduinobotTaskAction, ArduinobotTaskResult
import sys
import moveit_commander

"""
  arduinobot - task_server

  This script implements an Action Server that manages the execution
  of goals of the robot interfacing with moveit.
  Given a goal, it sends and execute a moveit trajectory

  Copyright (c) 2021 Antonio Brandi.  All right reserved.
"""


class TaskServer(object):
    # create messages that are used to publish feedback/result
    result_ = ArduinobotTaskResult()
    arm_goal_ = []
    gripper_goal_ = []

    def __init__(self, name):
        # Constructor
        # function that inizialize the ArduinobotTaskAction class and creates 
        # a Simple Action Server from the library actionlib
        # Constructor that gets called when a new instance of this class is created
        # it basically inizialize the MoveIt! API that will be used throughout the script
        # initialize the ROS interface with the robot via moveit
        moveit_commander.roscpp_initialize(sys.argv)

        # create a move group commander object that will be the interface with the robot joints
        self.arm_move_group_ = moveit_commander.MoveGroupCommander('arduinobot_arm')

        # create a move group commander object for the gripper
        self.gripper_move_group_ = moveit_commander.MoveGroupCommander('arduinobot_hand')

        self.action_name_ = name
        self.as_ = actionlib.SimpleActionServer(self.action_name_, ArduinobotTaskAction, execute_cb=self.execute_cb, auto_start = False)
        self.as_.start()
      

    def execute_cb(self, goal):

        success = True     

        # start executing the action
        # based on the goal id received, send a different goal 
        # to the robot
        if goal.task_number == 0:
            self.arm_goal_ = [0.0,0.0,0.0]
            self.gripper_goal_ = [-0.7, 0.7]
        elif goal.task_number == 1:
            self.arm_goal_ = [-1.14, -0.6, -0.07]
            self.gripper_goal_ = [0.0, 0.0]
        elif goal.task_number == 2:
            self.arm_goal_ = [-1.57,0.0,-1.0]
            self.gripper_goal_ = [0.0, 0.0]
        else:
            rospy.logerr('Invalid goal')
            return

        # Sends a goal to the moveit API
        self.arm_move_group_.go(self.arm_goal_, wait=True)
        self.gripper_move_group_.go(self.gripper_goal_, wait=True)

        # Make sure that no residual movement remains
        self.arm_move_group_.stop()
        self.gripper_move_group_.stop()

        # check that preempt has not been requested by the client
        if self.as_.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.action_name_)
            self.as_.set_preempted()
            success = False
       
        # check if the goal request has been executed correctly
        if success:
            self.result_.success = True
            rospy.loginfo('%s: Succeeded' % self.action_name_)
            self.as_.set_succeeded(self.result_)        


if __name__ == '__main__':

    # Inizialize a ROS node called task_server
    rospy.init_node('task_server')

    server = TaskServer(rospy.get_name())

    # keeps the node up and running
    rospy.spin()
