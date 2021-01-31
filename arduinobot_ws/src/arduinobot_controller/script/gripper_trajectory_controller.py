#! /usr/bin/env python
import rospy
import actionlib
import math
import sys
import control_msgs.msg
from std_msgs.msg import UInt16, Float64
from arduinobot_controller.srv import AnglesConverter


"""
  arduinobot - gripper_controller

  This file creates an Action Server that is in charge of receive and execute GripperCommand and track
  their execution giving feedback during its execution and a result after its completion.
  The current node is parametric and will work with both real robot controlled via Arduino and
  simulated robots in Gazebo

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""

# constant that defines the start pose of the robot
START_POSE = 0.0
JOINT_NAMES = ['joint_4']


class GripperControllerAction(object):
    # create messages that are used to publish feedback/result during the action execution
    # and after its completion
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name):
        # Constructor, called when an instance of this class is created
        # It init and starts an action server with the GripperCommandAction interface
        # It sends and executes the start pose of the robot gripper so that the start configuration of 
        # the robot gripper is known
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.goal_cb, auto_start = False)
        self._as.start()
        self.old_joint_angle = START_POSE
        self.execute(self.old_joint_angle)
      
    
    def goal_cb(self, goal):
        # This function is called when the action server receives a new goal
        # The received goal is a GripperCommandActionGoal message and contains
        # the list of position that the gripper joint should follow.
        success = True
        rospy.loginfo('%s: Gripper Action Received' % self._action_name)
        
        # Loop that executes each pose of the robot in the given goal trajectory
        # delaying each consequent pose by a given delay 
        for i, point in enumerate(goal.trajectory.points):

            # At each execution of the loop, check if the action server received a cancelation request
            # of the ongoing goal. If so, interrupt the execution of the goal and 
            # return a result message with status failed
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            # wait before starting the execution of the next goal
            delay = point.time_from_start - goal.trajectory.points[i-1].time_from_start if i>0 else 0
            rospy.sleep(delay)

            # reach the current loop pose
            self.execute(point.positions)

            # Fill out a FollowJointTrajectoryFeedback message to provide
            # a feedback about the current goal execution
            self._feedback.joint_names = JOINT_NAMES
            self._feedback.actual = point
            self._feedback.desired = point
            self._as.publish_feedback(self._feedback)

        # if during its execution the goal hasn't received any cancelation request,
        # return a result message with status succeeded
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)         
        

    def execute(self, angle):
        # This function checks if the robot is real or simulated 
        # and publishes the target pose on the robot on the matching topic
        rospy.loginfo('Angle Radians: %s' % angle)
        radians_to_degrees = rospy.ServiceProxy('/radians_to_degrees', AnglesConverter)
        angles_deg = radians_to_degrees(0,0,0,angle[0])
        rospy.loginfo('Angle Degrees: %s' % angles_deg.gripper)
        pub.publish(data=int(angles_deg.gripper))
        

if __name__ == '__main__':
    # Inizialize a ROS node called gripper_action
    rospy.init_node('gripper_controller')

    pub = rospy.Publisher('/arduino/gripper_actuate', UInt16, queue_size=10)

    rospy.wait_for_service('/radians_to_degrees')

    # Init the FollowJointTrajectory action server that will receive a trajectory for each joint and will
    # execute it in the real robot
    server = GripperControllerAction(rospy.get_name())

    # keep this ROS node up and running
    rospy.spin()
