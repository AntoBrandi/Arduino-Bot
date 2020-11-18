#! /usr/bin/env python
import rospy
import actionlib
import math
import sys
import control_msgs.msg
from std_msgs.msg import UInt16, Float64


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


class GripperControllerAction(object):
    # create messages that are used to publish feedback/result during the action execution
    # and after its completion
    _feedback = control_msgs.msg.GripperCommandFeedback()
    _result = control_msgs.msg.GripperCommandResult()

    def __init__(self, name):
        # Constructor, called when an instance of this class is created
        # It init and starts an action server with the GripperCommandAction interface
        # It sends and executes the start pose of the robot gripper so that the start configuration of 
        # the robot gripper is known
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, execute_cb=self.goal_cb, auto_start = False)
        self._as.start()
        self.old_joint_angle = START_POSE
        self.execute(self.old_joint_angle)
      
    
    def goal_cb(self, goal):
        # This function is called when the action server receives a new goal
        # The received goal is a GripperCommandActionGoal message and contains
        # the list of position that the gripper joint should follow.
        success = True
        rospy.loginfo('%s: Gripper Action Received' % self._action_name)
        
        # Before the execution, check if the action server received a cancelation request
        # of the ongoing goal. If so, interrupt the execution of the goal and 
        # return a result message with status failed
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        else:
            # reach the target pose
            self.execute(goal.command.position)

            # Fill out a GripperCommandFeedback message to provide
            # a feedback about the current goal execution
            self._feedback.position = goal.command.position
            self._feedback.reached_goal = True
            self._as.publish_feedback(self._feedback)

        # if during its execution the goal hasn't received any cancelation request,
        # return a result message with status succeeded
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result.position = goal.command.position
            self._result.reached_goal = True
            self._as.set_succeeded(self._result)         
        

    def execute(self, angle):
        # This function checks if the robot is real or simulated 
        # and publishes the target pose on the robot on the matching topic
        rospy.loginfo('Angle Radians: %s' % angle)
        # The trajectory controller is moving a simulated robot
        if isSimulated:
            pub.publish(data=angle)
        # The trajectory controller is moving a real robot controlled by Arduino
        else:
            angle_deg = self.convert_gripper_angle(angle)
            rospy.loginfo('Angle Degrees: %s' % angle_deg)
            pub.publish(data=int(angle_deg))

    def convert_gripper_angle(self, angle_rad):
        # Converts the angle of the gripper joint from radians to degrees
        return int(((-angle_rad)*180)/(math.pi/2))
        

if __name__ == '__main__':
    # Inizialize a ROS node called gripper_action
    rospy.init_node('gripper_action')

    # get the parameters passed to this node when is launched
    isSimulated = rospy.get_param('~is_simulated')

    # Accoring to the input parameter is_simulated, decide whether or not the robot is a real one controlled by Arduino
    # or is a simulated one in Gazebo. The publisher topic will be chosen accordingly 
    pub = rospy.Publisher('arduino_sim/gripper_actuate', Float64, queue_size=10) if isSimulated else rospy.Publisher('arduino/gripper_actuate', UInt16, queue_size=10)

    # Init the FollowJointTrajectory action server that will receive a trajectory for each joint and will
    # execute it in the real robot
    server = GripperControllerAction(rospy.get_name())

    # keep this ROS node up and running
    rospy.spin()
