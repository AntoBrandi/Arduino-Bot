#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from moveit_interface import MoveitInterface

"""
  arduinobot - pick_task

  This script implements a sample Task that the robot might implement and execute.
  When triggered, the execution of the task is started by using the moveit API for python
  which calculates a new execution plan and executes it
  

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""


class PickTask:

    # container of the current task positions
    positions = []
    # Set a list of possible joint configuration of the robot
    # JOINT A
    position_a = [1.49921669110568, 0.0, -0.39521235582288017, -1.0, 1.0]

    # JOINT B
    position_b = [1.49921669110568, -0.5978450819800801, -0.39521235582288017, -1.0, 1.0]

    # JOINT C 
    position_c = [1.49921669110568, -0.5978450819800801, -0.39521235582288017, 0.0, 0.0]

    # JOINT D 
    position_d = [1.49921669110568, -0.5978450819800801, 0.8, 0.0, 0.0]

    # JOINT E
    position_e = [1.49921669110568, 0.0, 0.8, 0.0, 0.0]

    # JOINT F
    position_f = [-1.1451105222372, -0.5978450819800801, -0.07099999397135992, 0.0, 0.0]

    def __init__(self):
        # Constructor that is calld when a new instance of this class is created
        # it inizialize the positions list that contains an ordered list of joint 
        # position to be followed
        self.positions.append(self.position_a)
        self.positions.append(self.position_b)
        self.positions.append(self.position_c)
        self.positions.append(self.position_d)
        self.positions.append(self.position_e)
        self.positions.append(self.position_f)

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

