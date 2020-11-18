#! /usr/bin/env python
import rospy
import actionlib
import control_msgs.msg
import math
from std_msgs.msg import UInt16MultiArray, Float64MultiArray

"""
  arduinobot - trajectory_controller

  This file creates an Action Server that is in charge of receiving and executing FollowJointTrajectory and track
  their execution giving feedback during its execution and a result after its completion.
  The current node is parametric and will work with both real robot controlled via Arduino and
  simulated robots in Gazebo

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""

# constant list of the joint names for the arm
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3']

# constant that defines the start pose of the robot
START_POSE = [0.0,0.0,0.0]


class TrajectoryControllerAction(object):
    # create messages that are used to publish feedback/result during the action execution
    # and after its completion
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name):
        # Constructor, called when an instance of this class is created
        # It init and starts an action server with the FollowJointTrajectoryAction interface
        # It sends and executes the start pose of the robot so that the start configuration of 
        # the robot is known
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.goal_cb, auto_start = False)
        self._as.start()
        self.old_joint_angles = START_POSE
        self.execute(self.old_joint_angles, JOINT_NAMES)
      
    def goal_cb(self, goal):
        # This function is called when the action server receives a new goal
        # The received goal is a FollowJointTrajectoryActionGoal message and contains
        # the list of position that each joint should follow. Each of this consequent poses of the 
        # joints are separated by a given time interval. 
        success = True
        rospy.loginfo('%s: Goal Received' % self._action_name)
        joint_names = goal.trajectory.joint_names
        
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
            self.execute(point.positions, joint_names)

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


    def convert_angles(self, angles, joint_names):
        # This function receives an array of joint angles in radians and
        # convert those in degrees in order to be appliend to the servo motors
        # by the Arduino controller respecting its boundaries
        current_angles = []
        for i, joint_name in enumerate(JOINT_NAMES):
            if joint_name in joint_names:

                if i == 0:
                    current_angles.append(self.convert_base_angle(angles[i]))
                elif i == 1:
                    current_angles.append(self.convert_shoulder_angle(angles[i]))
                else:
                    current_angles.append(self.convert_elbow_angle(angles[i]))

                self.old_joint_angles[i] = current_angles[i]
            else:
                current_angles.append(self.old_joint_angles[i])

        return current_angles           
        

    def execute(self, angles, joint_names):
        # This function checks if the robot is real or simulated 
        # and publishes the target pose on the robot on the matching topic
        rospy.loginfo('Angles Radians : %s' % str(angles))
        # The trajectory controller is moving a simulated robot
        if isSimulated:
            pub.publish(data=angles)
        # The trajectory controller is moving a real robot controlled by Arduino
        else:
            angles_deg = self.convert_angles(angles, joint_names)
            rospy.loginfo('Angles Degrees : %s' % str(angles_deg))
            pub.publish(data=angles_deg)
    
    def convert_base_angle(self, angle_rad):
        # Converts the angle of the base joint from radians to degrees
        return int(((angle_rad+(math.pi/2))*180)/math.pi)
    
    def convert_shoulder_angle(self, angle_rad):
        # Converts the angle of the shoulder joint from radians to degrees
        return 180-int(((angle_rad+(math.pi/2))*180)/math.pi)

    def convert_elbow_angle(self, angle_rad):
        # Converts the angle of the elbow joint from radians to degrees
        return int(((angle_rad+(math.pi/2))*180)/math.pi)
        

if __name__ == '__main__':
    # Inizialize a ROS node called trajectory_action
    rospy.init_node('trajectory_action')

    # get the parameters passed to this node when is launched
    isSimulated = rospy.get_param('~is_simulated')

    # Accoring to the input parameter is_simulated, decide whether or not the robot is a real one controlled by Arduino
    # or is a simulated one in Gazebo. The publisher topic will be chosen accordingly 
    pub = rospy.Publisher('arduino_sim/arm_actuate', Float64MultiArray, queue_size=10) if isSimulated else rospy.Publisher('arduino/arm_actuate', UInt16MultiArray, queue_size=10)
        
    # Init the FollowJointTrajectory action server that will receive a trajectory for each joint and will
    # execute it in the real robot
    server = TrajectoryControllerAction(rospy.get_name())

    # keep this ROS node up and running
    rospy.spin()
