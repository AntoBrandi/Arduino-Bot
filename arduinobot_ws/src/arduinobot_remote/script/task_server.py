#! /usr/bin/env python
import rospy
import actionlib
from arduinobot_remote.msg import ArduinobotTaskAction, ArduinobotTaskGoal, ArduinobotTaskFeedback, ArduinobotTaskResult
from robot_actions import Wake, Sleep, Dance, Pick


class TaskServer(object):
    # create messages that are used to publish feedback/result
    _feedback = ArduinobotTaskFeedback()
    _result = ArduinobotTaskResult()

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
            wake = Wake()
            wake.run()
        elif goal.task_number == 1:
            dance = Dance()
            dance.run()
        elif goal.task_number == 2:
            pick = Pick()
            pick.run()
        elif goal.task_number == 3:
            sleep = Sleep()
            sleep.run()
        else:
            rospy.loginfo('Invalid goal')


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

    # Inizialize a ROS node called arduinobot_task
    rospy.init_node('task_server')

    server = TaskServer(rospy.get_name())

    # keeps the node up and running
    rospy.spin()
