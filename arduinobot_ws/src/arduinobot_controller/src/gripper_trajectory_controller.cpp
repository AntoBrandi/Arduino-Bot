/*
    arduinobot - gripper_controller

    This file creates an Action Server that is in charge of receive and execute GripperCommand and track
    their execution giving feedback during its execution and a result after its completion.
    The current node is parametric and will work with both real robot controlled via Arduino and
    simulated robots in Gazebo

    Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
#include "arduinobot_controller/AnglesConverter.h"

// constant that defines the start pose of the robot
double const START_POSE = 0.0;

ros::Publisher pub;
ros::ServiceClient client;

class GripperControllerAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to publish feedback/result during the action execution
    // and after its completion
    control_msgs::GripperCommandFeedback feedback_;
    control_msgs::GripperCommandResult result_;
    double old_joint_angle_;

public:
    GripperControllerAction(std::string name) : as_(nh_, name, boost::bind(&GripperControllerAction::goal_cb, this, _1), false),
                                                action_name_(name)
    {
        old_joint_angle_ = START_POSE;

        nh_ = ros::NodeHandle();

        // Accoring to the input parameter is_simulated, decide whether or not the robot is a real one controlled by Arduino
        // or is a simulated one in Gazebo. The publisher topic will be chosen accordingly
        pub = nh_.advertise<std_msgs::UInt16>("/arduino/gripper_actuate", 1000);

        client = nh_.serviceClient<arduinobot_controller::AnglesConverter>("/radians_to_degrees");

        as_.start();
        execute(START_POSE);
    }

    ~GripperControllerAction(void)
    {
    }

    void goal_cb(const control_msgs::GripperCommandGoalConstPtr &goal)
    {
        // This function is called when the action server receives a new goal
        // The received goal is a GripperCommandActionGoal message and contains
        // the list of position that the gripper joint should follow
        bool success = true;
        // publish info to the console for the user
        ROS_INFO("%s: Gripper Action Received", action_name_.c_str());

        // Before the execution, check if the action server received a cancelation request
        // of the ongoing goal. If so, interrupt the execution of the goal and 
        // return a result message with status failed
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
        }
        else
        {
            // reach the target pose
            execute(goal->command.position);

            // Fill out a GripperCommandFeedback message to provide
            // a feedback about the current goal execution
            feedback_.position = goal->command.position;
            feedback_.reached_goal = true;
            as_.publishFeedback(feedback_);
        }

        // if during its execution the goal hasn't received any cancelation request,
        // return a result message with status succeeded
        if (success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            result_.position = goal->command.position;
            result_.reached_goal = true;
            as_.setSucceeded(result_);
        }
    }

    void execute(double angle)
    {
        // This function checks if the robot is real or simulated 
        // and publishes the target pose on the robot on the matching topic
        ROS_INFO("Angle Radians: %f", angle);

        std_msgs::UInt16 msg;
        // compose the service request message
        arduinobot_controller::AnglesConverter srv;
        srv.request.base = 0;
        srv.request.shoulder = 0;
        srv.request.elbow = 0;
        srv.request.gripper = angle;

        // Call the service and show the response of the service
        if (client.call(srv))
        {
            // compose the message
            msg.data = static_cast<unsigned int>(srv.response.gripper);

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
    // Inizialize a ROS node called gripper_action
    ros::init(argc, argv, "gripper_controller");

    // Init the FollowJointTrajectory action server that will receive a trajectory for each joint and will
    // execute it in the real robot
    GripperControllerAction server("gripper_controller");

    // keep this ROS node up and running
    ros::spin();
    return 0;
}