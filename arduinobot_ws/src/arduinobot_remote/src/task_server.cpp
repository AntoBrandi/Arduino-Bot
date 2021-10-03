#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <arduinobot_remote/ArduinobotTaskAction.h>
#include <sensor_msgs/JointState.h>
#include "moveit_interface.cpp"


class TaskServer
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<arduinobot_remote::ArduinobotTaskAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  arduinobot_remote::ArduinobotTaskFeedback feedback_;
  arduinobot_remote::ArduinobotTaskResult result_;
  std::vector<double> arm_goal_;
  std::vector<double> gripper_goal_;
  MoveitInterface moveit_;

public:

  TaskServer(std::string name) :
    as_(nh_, name, boost::bind(&TaskServer::execute_cb, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  void execute_cb(const arduinobot_remote::ArduinobotTaskGoalConstPtr &goal)
  {
    bool success = true;

    if (goal->task_number == 0)
    {
      arm_goal_ = {0.0, 0.0, 0.0};
      gripper_goal_ = {-0.7, 0.7};
    }
    else if (goal->task_number == 1)
    {
      arm_goal_ = {-1.14, -0.6, -0.07};
      gripper_goal_ = {0.0, 0.0};
    }
    else if (goal->task_number == 2)
    {
      arm_goal_ = {-1.57,0.0,-1.0};
      gripper_goal_ = {0.0, 0.0};
    }
    else
    {
        ROS_ERROR("Invalid goal");
    }

    moveit_.set_max_velocity(0.7);
    moveit_.set_max_acceleration(0.1);
    moveit_.reach_goal(arm_goal_, gripper_goal_);

    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
    }

    if(success)
    {
      result_.success = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  // Inizialize a ROS node called task_server
  ros::init(argc, argv, "task_server");
  TaskServer server("task_server");

  // keeps the node up and running
  ros::spin();
  return 0;
}