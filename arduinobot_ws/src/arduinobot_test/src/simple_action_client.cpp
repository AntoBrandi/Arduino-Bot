#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arduinobot_test/FibonacciAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "fibonacci_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<arduinobot_test::FibonacciAction> ac("fibonacci");

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  arduinobot_test::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);

  //wait for the action to return
  ac.waitForResult(ros::Duration(30.0));

  arduinobot_test::FibonacciResultConstPtr result = ac.getResult();
  ROS_INFO("Action finished:");
  for (int i : result->sequence)
  {
    ROS_INFO("%d", i);
  }
  
  //exit
  return 0;
}