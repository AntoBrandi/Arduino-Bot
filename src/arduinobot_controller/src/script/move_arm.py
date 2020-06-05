#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# initialize the moveit_commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_arm',anonymous=True)

# instantiate a robot commander object that provides information 
# about the robot's kinematic model and robot's current joint state
robot = moveit_commander.RobotCommander()

# instantiate a planning scene interface object that provides a remote interface for 
# getting, setting and updating the robot internal understanding of the world
scene = moveit_commander.PlanningSceneInterface()

# instantiate a moveit commander object that is an interface to a planning group
# or to a group of joint
group_name = "arduinobot_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# instantiate a display trajectory ros publisherthat is used to display
# trajectories in Rviz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# plan a joint goal
# publish a position target in the joint space
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0

move_group.go(joint_goal,wait=True)
move_group.stop()
