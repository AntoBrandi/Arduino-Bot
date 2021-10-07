#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import actionlib_tutorials.msg


def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

    # Sends the goal to the action server.
    print("Seding goal order %s" % goal.order)
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  


if __name__ == '__main__':
    try:
        # Inizialize a ROS node called fibonacci_client
        rospy.init_node('fibonacci_client')

        print("Simple Action Client Started")

        # Keep this node active until the action server returns a result
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)