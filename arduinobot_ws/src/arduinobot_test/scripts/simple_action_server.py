#! /usr/bin/env python3
import rospy
import actionlib
from arduinobot_test.msg import FibonacciAction, FibonacciResult, FibonacciFeedback


class FibonacciActionServer(object):
    # create messages that are used to publish feedback/result
    feedback_ = FibonacciFeedback()
    result_ = FibonacciResult()

    def __init__(self, name):
        # Constructor
        # function that inizialize the FibonacciAction class and creates 
        # a Simple Action Server from the library actionlib
        self.action_name_ = name
        self.as_ = actionlib.SimpleActionServer(self.action_name_, FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self.as_.start()
        rospy.loginfo('Simple Action Server Started')
      

    def execute_cb(self, goal):
        rospy.loginfo('Goal Received : %s' % goal.order)

        # Function that is called each time the action server receives a new goal
        # It starts displaying a fibonacci series from 1 to the goal target
        r = rospy.Rate(1)
        success = True
        
        # Sends an action feedback 
        self.feedback_.sequence = []
        self.feedback_.sequence.append(1)
        self.feedback_.sequence.append(1)
        

        # start executing the action
        for i in range(1, (goal.order - 1)):
            # check that preempt has not been requested by the client
            if self.as_.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.action_name_)
                self.as_.set_preempted()
                success = False
                break
            self.feedback_.sequence.append(self.feedback_.sequence[i] + self.feedback_.sequence[i-1])
            # publish the feedback
            self.as_.publishfeedback_(self.feedback_)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        # If no cancelation requests are received, return a suceeded result 
        if success:
            self.result_.sequence = self.feedback_.sequence
            rospy.loginfo('%s: Succeeded' % self.action_name_)
            self.as_.set_succeeded(self.result_)


if __name__ == '__main__':

    # Inizialize a ROS node called fibonacci
    rospy.init_node('fibonacci')

    # create an instance of the class FibonacciAction which will inizialize
    # a Simple Action Server from the actionlib library called finonacci
    server = FibonacciActionServer(rospy.get_name())

    # keeps the node up and running
    rospy.spin()
