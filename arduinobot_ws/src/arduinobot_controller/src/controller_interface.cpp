/*
    arduinobot - controller_interface

    This script implements an interface between ROS Master on the PC/Raspberry side and the
    Arduino Controller with a ROS node that actuates the servo motors

    The aim of the script is to convert std_msgs/UInt16MultiArray messages received from Arduino into 
    a sensor_msgs/JointState message that is the one required in ROS for the presentation
    of the joint status related data and that can be used in other ROS components
    The order of each joint in the received message is the following

    - base
    - shoulder
    - elbow
    - gripper

    The Arduino is publishing std_msgs/UInt16MultiArray messages containing the current status of each joint
    in the previous order instead of publishing directly a sensor_msgs/JointState message because is lighter

    Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "arduinobot_controller/AnglesConverter.h"
#include "vector"
#include "string"


ros::Publisher pub_status;
ros::Subscriber sub;
ros::ServiceClient client;
std::vector<std::string> JOINT_NAMES;


void joint_states_cb(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
    //  Callback function that is called each time a new message is published on the topic /arduinobot/joint_states
    //  the received message is passed as input to this function
    //  It filles out a new JointState messages using the data contained in the received UInt16MultiArray message
    //  and then publishes the composed JointState message on the topic /arduinobot/joint_states
    sensor_msgs::JointState status;
    status.header.stamp = ros::Time::now();
    status.name = JOINT_NAMES;

    // compose the service request message
    arduinobot_controller::AnglesConverter srv;
    srv.request.base = msg->data[0];
    srv.request.shoulder = msg->data[1];
    srv.request.elbow = msg->data[2];
    srv.request.gripper = msg->data[3];

    // Call the service and show the response of the service
    if (client.call(srv))
    {
        // compose the array message
        std::vector<double> angles_rad;
        angles_rad.push_back(srv.response.base);
        angles_rad.push_back(srv.response.shoulder);
        angles_rad.push_back(srv.response.elbow);
        angles_rad.push_back(srv.response.gripper);

        status.position = angles_rad;

        // publish the array message to the defined topic
        pub_status.publish(status);
    }
    else
    {
        ROS_ERROR("Failed to call service degrees_to_radians");
    }
}

int main(int argc, char **argv)
{
    // Inizialize a ROS node called controller_interface
    ros::init(argc, argv, "controller_interface");
    ros::NodeHandle n;

    JOINT_NAMES.push_back("joint_1");
    JOINT_NAMES.push_back("joint_2");
    JOINT_NAMES.push_back("joint_3");
    JOINT_NAMES.push_back("joint_4");

    // register a publisher on the topic /arduinobot/joint_states that will publish JointState messages
    pub_status = n.advertise<sensor_msgs::JointState>("arduinobot/joint_states", 1000);

    // register a subscriber on the topic /arduino/joint_states that will listen for UInt16MultiArray messages
    // when a new message is received, the callback function is triggered and starts its execution
    sub = n.subscribe("arduino/joint_states", 1000, joint_states_cb);

    client = n.serviceClient<arduinobot_controller::AnglesConverter>("/degrees_to_radians");

    // keeps the node up and running
    ros::spin();
    return 0;
}