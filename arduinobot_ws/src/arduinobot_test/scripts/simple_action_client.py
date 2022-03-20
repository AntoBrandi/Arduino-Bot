#! /usr/bin/env python3
import rospy
import actionlib
from arduinobot_test.msg import FibonacciAction, FibonacciGoal


def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci', FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = FibonacciGoal(order=20)

    # Sends the goal to the action server.
    print("Seding goal order %s" % goal.order)
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  


if __name__ == '__main__':
    # Inizialize a ROS node called fibonacci_client
    rospy.init_node('fibonacci_client')

    print("Simple Action Client Started")

    # Keep this node active until the action server returns a result
    result = fibonacci_client()
    print("Result: ", result.sequence)