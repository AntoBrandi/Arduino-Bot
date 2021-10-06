#! /usr/bin/env python
import rospy
import actionlib
from arduinobot_remote.msg import ArduinobotTaskAction, ArduinobotTaskFeedback, ArduinobotTaskResult
from sensor_msgs.msg import JointState
from moveit_interface import MoveitInterface

"""
  arduinobot - task_server

  This script implements an Action Server that manages the execution
  of goals of the robot interfacing with moveit.
  Given a goal, it sends and execute a moveit trajectory

  Copyright (c) 2021 Antonio Brandi.  All right reserved.
"""


class TaskServer(object):
    # create messages that are used to publish feedback/result
    _feedback = ArduinobotTaskFeedback()
    _result = ArduinobotTaskResult()
    _moveit = MoveitInterface()
    _arm_goal = []
    _gripper_goal = []

    def __init__(self, name):
        # Constructor
        # function that inizialize the ArduinobotTaskAction class and creates 
        # a Simple Action Server from the library actionlib
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ArduinobotTaskAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      

    def execute_cb(self, goal):

        success = True     

        # start executing the action
        # based on the goal id received, send a different goal 
        # to the robot
        if goal.task_number == 0:
            self._arm_goal = [0.0,0.0,0.0]
            self._gripper_goal = [-0.7, 0.7]
        elif goal.task_number == 1:
            self._arm_goal = [-1.14, -0.6, -0.07]
            self._gripper_goal = [0.0, 0.0]
        elif goal.task_number == 2:
            self._arm_goal = [-1.57,0.0,-1.0]
            self._gripper_goal = [0.0, 0.0]
        else:
            rospy.logerr('Invalid goal')

        # Sends a goal to the moveit API
        self._moveit.set_max_velocity(0.7)
        self._moveit.set_max_acceleration(0.1)
        self._moveit.reach_goal(self._arm_goal, self._gripper_goal)

        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
       
        # check if the goal request has been executed correctly
        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)        


if __name__ == '__main__':

    # Inizialize a ROS node called task_server
    rospy.init_node('task_server')

    server = TaskServer(rospy.get_name())

    # keeps the node up and running
    rospy.spin()
