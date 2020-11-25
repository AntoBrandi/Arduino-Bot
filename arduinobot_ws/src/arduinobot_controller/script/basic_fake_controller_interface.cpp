/*
    arduinobot - basic_fake_controller_interface

    This script implements an interface between ROS Master on the PC/Raspberry side and the
    gazebo fake controllers that actuates the robot

    The aim of the script is to convert sensor_msgs/JointState messages into 
    a std_msgs/Float64 messages for each joint of the robot
    The order of each joint in the array is the following

    - base
    - shoulder
    - elbow
    - gripper

    Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include "sensor_msgs/JointState.h"


ros::Publisher pub_joint_1;
ros::Publisher pub_joint_2;
ros::Publisher pub_joint_3;
ros::Publisher pub_joint_4;
ros::Publisher pub_joint_5;


void callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // This function is called each time a new message is published on the topic /joint_states
    // this message is a sensor_msg/JointStatmessage type
    // get the data and publish those on the connected arduino as joint angles
    // The message that was published on the topic /joint_states is then passed to this function as input
    pub_joint_1.publish(msg->position[0]);
    pub_joint_2.publish(msg->position[1]);
    pub_joint_3.publish(msg->position[2]);
    pub_joint_4.publish(msg->position[3]);
    pub_joint_5.publish(msg->position[4]);
}


int main(int argc, char **argv)
{
    // Inizialize a ROS node called basic_fake_controller_interface
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    // register the publishers for the position controller topics that are publishing Float64 messages
    // which indicates the angle in radians that a given joint will be actuated
    pub_joint_1 = n.advertise<std_msgs::Float64>("arduinobot_sim/joint_1_position_controller/command", 1000);
    pub_joint_2 = n.advertise<std_msgs::Float64>("arduinobot_sim/joint_2_position_controller/command", 1000);
    pub_joint_3 = n.advertise<std_msgs::Float64>("arduinobot_sim/joint_3_position_controller/command", 1000);
    pub_joint_4 = n.advertise<std_msgs::Float64>("arduinobot_sim/joint_4_position_controller/command", 1000);
    pub_joint_5 = n.advertise<std_msgs::Float64>("arduinobot_sim/joint_5_position_controller/command", 1000);

    // register a subscriber on the topic /joint_states that will listen for JointState messages
    // when a new message is received, the callback function is triggered and starts its execution
    ros::Subscriber sub = n.subscribe("joint_states", 1000, callback);

    // keep this ROS node up and running
    ros::spin();
    return 0;
}