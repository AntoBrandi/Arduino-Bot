/*
    arduinobot - trajectory_controller

    This file creates an Action Server that is in charge of receiving and executing FollowJointTrajectory and track
    their execution giving feedback during its execution and a result after its completion.
    The current node is parametric and will work with both real robot controlled via Arduino and
    simulated robots in Gazebo

    Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "arduinobot_controller/AnglesConverter.h"
#include "vector"


std::vector<double> START_POSE;
std::vector<std::string> JOINT_NAMES;

ros::Publisher pub;
ros::ServiceClient client;
bool is_simulated = true;


class TrajectoryControllerAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;

  public:
  TrajectoryControllerAction(std::string name) : as_(nh_, name, boost::bind(&TrajectoryControllerAction::goal_cb, this, _1), false),
                                                 action_name_(name)
  {
      START_POSE.push_back(0.0);
      START_POSE.push_back(0.0);
      START_POSE.push_back(0.0);
      JOINT_NAMES.push_back("joint_1");
      JOINT_NAMES.push_back("joint_2");
      JOINT_NAMES.push_back("joint_3");

      nh_ = ros::NodeHandle("~");
      // Constructor, called when an instance of this class is created
      // It init and starts an action server with the GripperCommandAction interface
      // It sends and executes the start pose of the robot gripper so that the start configuration of
      // the robot gripper is known
      // get the parameters passed to this node when is launched
      nh_.getParam("is_simulated", is_simulated);

      old_joint_angle_ = START_POSE;

      nh_ = ros::NodeHandle();

      // Accoring to the input parameter is_simulated, decide whether or not the robot is a real one controlled by Arduino
      // or is a simulated one in Gazebo. The publisher topic will be chosen accordingly
      pub = is_simulated ? nh_.advertise<std_msgs::Float64MultiArray>("arduino_sim/arm_actuate", 1000) : nh_.advertise<std_msgs::UInt16MultiArray>("arduino/arm_actuate", 1000);

      if (!is_simulated)
      {
          client = nh_.serviceClient<arduinobot_controller::AnglesConverter>("radians_to_degrees");
      }

      as_.start();
      execute(START_POSE);
  }

  ~TrajectoryControllerAction(void)
  {
  }

  void goal_cb(const control_msgs::TrajectoryControllerGoalConstPtr &goal)
  {
      bool success = true;

      ROS_INFO("%s:  Goal Received", action_name_.c_str());

      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          success = false;
      }
      else
      {

      }

      if (success)
      {
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          // set the action state to succeeded
          as_.setSucceeded(result_);
      }
  }

  void execute(std::vector<double> angles)
  {
      ROS_INFO("Angle Radians: %f %f %f", angles.at(0), angles.at(1), angles.at(2));
      // The trajectory controller is moving a simulated robot
      // The trajectory controller is moving a simulated robot
      if (is_simulated)
      {
          std_msgs::Float64MultiArray msg;
          // set up dimensions
          msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
          msg.layout.dim[0].size = angles.size();
          msg.layout.dim[0].stride = 1;

          // copy in the data
          msg.data.clear();
          msg.data.insert(msg.data.end(), angles.begin(), angles.end());
          
          pub.publish(msg);
      }
      // The trajectory controller is moving a real robot controlled by Arduino
      else
      {
          std_msgs::UInt16MultiArray msg;
          // compose the service request message
          arduinobot_controller::AnglesConverter srv;
          srv.request.base = angles.at(0);
          srv.request.shoulder = angles.at(1);
          srv.request.elbow = angles.at(2);
          srv.request.gripper = 0;

          // Call the service and show the response of the service
          if (client.call(srv))
          {
              // compose the message
              std::vector<double> angles_deg;
              angles_deg.push_back(static_cast<unsigned int>(srv.response.base));
              angles_deg.push_back(static_cast<unsigned int>(srv.response.shoulder));
              angles_deg.push_back(static_cast<unsigned int>(srv.response.elbow));

              // set up dimensions
              msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
              msg.layout.dim[0].size = angles_deg.size();
              msg.layout.dim[0].stride = 1;

              // copy in the data
              msg.data.clear();
              msg.data.insert(msg.data.end(), angles_deg.begin(), angles_deg.end());

              // publish the array message to the defined topic
              pub.publish(msg);
          }
          else
          {
              ROS_ERROR("Failed to call service radians_to_degrees");
          }
      }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_action");

  TrajectoryControllerAction server("trajectory_action");

  ros::spin();
  return 0;
}