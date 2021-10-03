#! /usr/bin/env python
import rospy
import actionlib
from arduinobot_remote.msg import ArduinobotTaskAction, ArduinobotTaskFeedback, ArduinobotTaskResult
from sensor_msgs.msg import JointState
from moveit_interface import MoveitInterface


class TaskServer(object):
    # create messages that are used to publish feedback/result
    _feedback = ArduinobotTaskFeedback()
    _result = ArduinobotTaskResult()
    _moveit = MoveitInterface()
    _goal = JointState()

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
        if goal.task_number == 0:
            self._goal.position = [0.0,0.0,0.0,-0.7, 0.7]
        elif goal.task_number == 1:
            self._goal.position = [-1.14, -0.6, -0.07, 0.0, 0.0]
        elif goal.task_number == 2:
            self._goal.position = [-1.57,0.0,-1.0,0.0, 0.0]
        else:
            rospy.logerr('Invalid goal')

        self._moveit.set_max_velocity(0.7)
        self._moveit.set_max_acceleration(0.1)
        self._moveit.reach_goal(self._goal.position[:-2], self._goal.position[-2:])

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
       
        # If no cancelation requests are received, return a suceeded result 
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
