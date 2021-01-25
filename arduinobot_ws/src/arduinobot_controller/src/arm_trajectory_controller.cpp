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

// constant that defines the start pose of the robot
std::vector<double> START_POSE;
// constant list of the joint names for the arm
std::vector<std::string> JOINT_NAMES;

ros::Publisher pub;
ros::ServiceClient client;


class TrajectoryControllerAction
{
    // create messages that are used to publish feedback/result during the action execution
    // and after its completion
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    control_msgs::FollowJointTrajectoryFeedback feedback_;
    control_msgs::FollowJointTrajectoryResult result_;
    std::vector<double> old_joint_angle_;

public:
    TrajectoryControllerAction(std::string name) : as_(nh_, name, boost::bind(&TrajectoryControllerAction::goal_cb, this, _1), false),
                                                   action_name_(name)
    {
        // Constructor, called when an instance of this class is created
        // It init and starts an action server with the FollowJointTrajectoryAction interface
        // It sends and executes the start pose of the robot so that the start configuration of 
        // the robot is known
        START_POSE.push_back(0.0);
        START_POSE.push_back(0.0);
        START_POSE.push_back(0.0);
        JOINT_NAMES.push_back("joint_1");
        JOINT_NAMES.push_back("joint_2");
        JOINT_NAMES.push_back("joint_3");


        old_joint_angle_ = START_POSE;

        nh_ = ros::NodeHandle();

        // Accoring to the input parameter is_simulated, decide whether or not the robot is a real one controlled by Arduino
        // or is a simulated one in Gazebo. The publisher topic will be chosen accordingly
        pub = nh_.advertise<std_msgs::UInt16MultiArray>("/arduino/arm_actuate", 1000);

        client = nh_.serviceClient<arduinobot_controller::AnglesConverter>("/radians_to_degrees");

        as_.start();
        execute(START_POSE);
    }

    ~TrajectoryControllerAction(void)
    {
    }

    void goal_cb(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
    {
        // This function is called when the action server receives a new goal
        // The received goal is a FollowJointTrajectoryActionGoal message and contains
        // the list of position that each joint should follow. Each of this consequent poses of the 
        // joints are separated by a given time interval. 
        bool success = true;

        ROS_INFO("%s:  Goal Received", action_name_.c_str());

        // Loop that executes each pose of the robot in the given goal trajectory
        // delaying each consequent pose by a given delay 
        for (int i = 0; i < goal->trajectory.points.size(); i++)
        {
            // At each execution of the loop, check if the action server received a cancelation request
            // of the ongoing goal. If so, interrupt the execution of the goal and 
            // return a result message with status failed
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }

            // wait before starting the execution of the next goal
            ros::Duration delay(0);
            if (i>0)
            {
                delay = goal->trajectory.points.at(i).time_from_start - goal->trajectory.points.at(i-1).time_from_start;
            } 
            delay.sleep();

            // reach the current loop pose
            execute(goal->trajectory.points.at(i).positions);

            // Fill out a FollowJointTrajectoryFeedback message to provide
            // a feedback about the current goal execution
            feedback_.joint_names = JOINT_NAMES;
            feedback_.actual = goal->trajectory.points.at(i);
            feedback_.desired = goal->trajectory.points.at(i);
            as_.publishFeedback(feedback_);
        }

        // if during its execution the goal hasn't received any cancelation request,
        // return a result message with status succeeded
        if (success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }

    void execute(std::vector<double> angles)
    {
        // This function checks if the robot is real or simulated
        // and publishes the target pose on the robot on the matching topic
        ROS_INFO("Angle Radians: %f %f %f", angles.at(0), angles.at(1), angles.at(2));
        // The trajectory controller is moving a simulated robot
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
};

int main(int argc, char **argv)
{
    // Inizialize a ROS node called trajectory_action
    ros::init(argc, argv, "arm_controller");

    // get the parameters passed to this node when is launched
    TrajectoryControllerAction server("arm_controller");

    // keep this ROS node up and running
    ros::spin();
    return 0;
}