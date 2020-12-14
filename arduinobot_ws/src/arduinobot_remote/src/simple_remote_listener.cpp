/*
    arduinobot - simple_remote_listener

    This script implements a simple subscriber on the topic /jarvis_messenger and prints
    all the received messages to the screen.
    It is a sample node whose aim is to show how it is possible to receive ROS messages
    that are published by external entities
    

    Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include "ros/ros.h"
#include "std_msgs/String.h"



void jarvis_messenger_callback(const std_msgs::String::ConstPtr& msg)
{
    // function that gets called every time a new message is published on the topic /jarvis_messenger
    // it prints the received String message in the console log
    ROS_INFO("Remote message received %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    // Inizialize a ROS node called simple_remote_listener
    ros::init(argc, argv, "simple_remote_listener");
    ros::NodeHandle n;

    // register a subscriber on the topic /jarvis_messenger that will listen for String messages
    // when a new message is received, the callback function is triggered and starts its execution
    ros::Subscriber sub = n.subscribe("jarvis_messenger", 1000, jarvis_messenger_callback);

    // keeps the node up and running
    ros::spin();
    return 0;
}