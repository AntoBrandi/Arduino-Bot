#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from moveit_interface import MoveitInterface

"""
  arduinobot - task

  This script implements a sample Task that the robot might implement and execute.
  When triggered, the execution of the task is started by using the moveit API for python
  which calculates a new execution plan and executes it
  

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""


class Task:

    def __init__(self):
        # Constructor that is calld when a new instance of this class is created
        # it inizialize the positions list that contains an ordered list of joint 
        # position to be followed
        # container of the current task positions
        self.positions = []

        # init the moveit interface for the execution of the current task
        self.moveit = MoveitInterface()

    
    def execute(self):
        # Execute the current task by asking the moveit plugin
        # to generate a trajectory that reaches each position of the robot, and then executes
        # the generated trajectory
        for i, position in enumerate(self.positions):
            rospy.loginfo('reaching position: %s', i)  
            goal = JointState()
            goal.position = position
            self.moveit.reach_goal(goal)

        self.positions = []


    def add_position(self, position):
        # Function that adds a new joint position to the joint positions list
        # This will keep track of a new position in the execution list
        self.positions.append(position)


    def set_speed(self, scaling_factor):
        # Function that sets the maximum speed of the joints as percentage
        # of the maximum speed defined in the joint_limits.yaml file
        self.moveit.set_max_velocity(scaling_factor)


    def set_acceleration(self, scaling_factor):
        # Function that sets the maximum acceleration of the joints as percentage
        # of the maximum speed defined in the joint_limits.yaml file
        self.moveit.set_max_acceleration(scaling_factor)




