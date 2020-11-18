#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

"""
  arduinobot - moveit_interface

  This script uses the MoveIt! API for Python in order to reach a given goal
  and eventually prints the current robot status and informations.

  When triggered, the reach_goal function uses the MoveIt! API for creating a planning request
  that generates a new trajectory that allows the robot to reach its new destination.
  Then, this trajectory is executed via controllers that have been registered and launched with MoveIt!

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""


# define the robot group names created with the MoveIt! Setup Assistant
ARM_GROUP_NAME = 'arduinobot_arm'
GRIPPER_GROUP_NAME = 'arduinobot_hand'

class MoveitInterface:

    def __init__(self):
        # Constructor that gets called when a new instance of this class is created
        # it basically inizialize the MoveIt! API that will be used throughout the script
        # initialize the ROS interface with the robot via moveit
        moveit_commander.roscpp_initialize(sys.argv)

        # create a robot commander object that will be the interface with the robot
        self.robot = moveit_commander.RobotCommander()

        # create a move group commander object that will be the interface with the robot joints
        self.arm_move_group = moveit_commander.MoveGroupCommander(ARM_GROUP_NAME)

        # create a move group commander object for the gripper
        self.gripper_move_group = moveit_commander.MoveGroupCommander(GRIPPER_GROUP_NAME)

        # create a display trajectory object that will publish the trajectory to rviz
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
                                                    

    def get_robot_info(self):
        # Function that prints the robot informations obtained via
        # MoveIt! APIs
        # We can get the name of the reference frame for this robot:
        planning_frame = self.arm_move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = self.arm_move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", group_names

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""

    
    def reach_goal(self, goal):
        # This function requires a JointState message as input 
        # with a list of 5 angles in radians for each joint
        # The first 3 elemnt of this list are passed to the arm group 
        # and the last 2 elements are passed to the gripper group.
        # Consider that the last two element of this list have to be the same 
        # as absolute value and with opposite sign
        arm_goal = goal.position[:-2]
        gripper_goal = goal.position[-2:]

        # Plan and Execute a trajectory that brings the robot from the current pose
        # to the target pose
        self.arm_move_group.go(arm_goal, wait=True)
        self.gripper_move_group.go(gripper_goal, wait=True)

        # Make sure that no residual movement remains
        self.arm_move_group.stop()
        self.gripper_move_group.stop()



