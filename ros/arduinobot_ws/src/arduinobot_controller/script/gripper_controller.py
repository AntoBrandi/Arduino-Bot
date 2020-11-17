#! /usr/bin/env python
import rospy
import actionlib
import math
import sys
import control_msgs.msg
from std_msgs.msg import UInt16, Float64


class GripperControllerAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.GripperCommandFeedback()
    _result = control_msgs.msg.GripperCommandResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, execute_cb=self.goal_cb, auto_start = False)
        self._as.start()
        self.old_joint_angle = 0.0
        self.execute(self.old_joint_angle)
      
    # This function is called when the action server receives a new goal
    def goal_cb(self, goal):
        success = True
        rospy.loginfo('%s: Gripper Action Received' % self._action_name)
        
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        else:
            self.execute(goal.command.position)

            self._feedback.position = goal.command.position
            self._feedback.reached_goal = True
            self._as.publish_feedback(self._feedback)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result.position = goal.command.position
            self._result.reached_goal = True
            self._as.set_succeeded(self._result)         
        

    def execute(self, angle):
        rospy.loginfo('Angle Radians: %s' % angle)
        if isSimulated:
            pub.publish(data=angle)
        else:
            angle_deg = self.convert_gripper_angle(angle)
            rospy.loginfo('Angle Degrees: %s' % angle_deg)
            pub.publish(data=int(angle_deg))

    def convert_gripper_angle(self, angle_rad):
        return int(((-angle_rad)*180)/(math.pi/2))
        

if __name__ == '__main__':

    rospy.init_node('gripper_action')

    # get the parameters passed to this node
    isSimulated = rospy.get_param('~is_simulated')

    # Publish the converted joint angles to the arduino subscriber node
    pub = rospy.Publisher('arduino_sim/gripper_actuate', Float64, queue_size=10) if isSimulated else rospy.Publisher('arduino/gripper_actuate', UInt16, queue_size=10)

    # Init the FollowJointTrajectory action server that will receive a trajectory for each joint and will
    # execute it in the real robot
    server = GripperControllerAction(rospy.get_name())
    rospy.spin()