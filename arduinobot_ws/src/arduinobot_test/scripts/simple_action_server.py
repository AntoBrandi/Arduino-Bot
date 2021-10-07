#! /usr/bin/env python
import rospy
import actionlib
import actionlib_tutorials.msg


class FibonacciAction(object):
    # create messages that are used to publish feedback/result
    _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        # Constructor
        # function that inizialize the FibonacciAction class and creates 
        # a Simple Action Server from the library actionlib
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('Simple Action Server Started')
      

    def execute_cb(self, goal):
        rospy.loginfo('Goal Received : %s' % goal.order)

        # Function that is called each time the action server receives a new goal
        # It starts displaying a fibonacci series from 1 to the goal target
        r = rospy.Rate(1)
        success = True
        
        # Sends an action feedback 
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        

        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        # If no cancelation requests are received, return a suceeded result 
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':

    # Inizialize a ROS node called fibonacci
    rospy.init_node('fibonacci')

    # create an instance of the class FibonacciAction which will inizialize
    # a Simple Action Server from the actionlib library called finonacci
    server = FibonacciAction(rospy.get_name())

    # keeps the node up and running
    rospy.spin()
