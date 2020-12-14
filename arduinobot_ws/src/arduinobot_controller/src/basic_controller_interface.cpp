/*
    arduinobot - basic_controller_interface

    This script implements an interface between ROS Master on the PC/Raspberry side and the
    Arduino Controller with a ROS node that actuates the servo motors

    The aim of the script is to convert sensor_msgs/JointState messages into 
    a std_msgs/UInt16MultiArray message that is lighter and so more suitable for 
    Arduino to be received and used.
    The order of each joint in the array is the following

    - base
    - shoulder
    - elbow
    - gripper

    Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "arduinobot_controller/AnglesConverter.h"
#include "vector"


ros::Publisher pub;
ros::Subscriber sub;
ros::ServiceClient client;


void callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // This function is called each time a new message is published on the topic /joint_states
    // this message is a sensor_msg/JointStatmessage type
    // get the data and publish those on the connected arduino as joint angles
    // The message that was published on the topic /joint_states is then passed to this function as input

    // compose the service request message
    arduinobot_controller::AnglesConverter srv;
    srv.request.base = msg->position[0];
    srv.request.shoulder = msg->position[1];
    srv.request.elbow = msg->position[2];
    srv.request.gripper = msg->position[3];

    // Call the service and show the response of the service
    if (client.call(srv))
    {
        // compose the array message
        std::vector<unsigned int> angles_deg;
        angles_deg.push_back(srv.response.base);
        angles_deg.push_back(srv.response.shoulder);
        angles_deg.push_back(srv.response.elbow);
        angles_deg.push_back(srv.response.gripper);

        std_msgs::UInt16MultiArray msg;

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


int main(int argc, char **argv)
{
    // Inizialize a ROS node called basic_controller_interface
    ros::init(argc, argv, "basic_controller_interface");
    ros::NodeHandle n;

    // register a publisher on the topic /servo_actuate that will publish UInt16MultiArray messages
    pub = n.advertise<std_msgs::UInt16MultiArray>("servo_actuate", 1000);

    // register a subscriber on the topic /joint_states that will listen for JointState messages
    // when a new message is received, the callback function is triggered and starts its execution
    sub = n.subscribe("joint_states", 1000, callback);

    client = n.serviceClient<arduinobot_controller::AnglesConverter>("radians_to_degrees");


    // keep this ROS node up and running
    ros::spin();
    return 0;
}