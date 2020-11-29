/*
    arduinobot - remote_controller

    This script implements an interface between the robot and the external world.
    It is in charge of receive messages from the external world, eventually validate them,
    and publish those on the correct topic of the robot where those will be used by other modules

    The open interface is listening on topics:

    - jarvis_messenger : Simple String messages are published. It may be useful 
                        for sending command to the robot via messages coming from different sources
                        (es. Telegram actuated robot)

    - jarvis_controller : JointState messages are published. It receives desidered poses of each joint 
                            of the robot and executes them

    - jarvis_voice : Bool messages are published. It receives a trigger for the execution of a pre recorded task
    

    Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"



void jarvis_controller_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // function that gets called every time a new message is published on the topic /jarvis_voice
    // triggers the execution of a pre-recorded task
}

void jarvis_messenger_callback(const std_msgs::String::ConstPtr& msg)
{
    // function that gets called every time a new message is published on the topic /jarvis_messenger
    // it prints the received String message in the console log
    ROS_INFO("Remote message received %s", msg->data.c_str());
}

void jarvis_voice_callback(const std_msgs::Bool::ConstPtr& msg)
{
    // function that gets called every time a new message is published on the topic /jarvis_voice
    // it start the execution of a previously recorded task
}

int main(int argc, char **argv)
{
    // Inizialize a ROS node called simple_remote_listener
    ros::init(argc, argv, "remote_controller");
    ros::NodeHandle n;

    // register a subscriber on the topic /jarvis_messenger that will listen for String messages
    // when a new message is received, the callback function is triggered and starts its execution
    ros::Subscriber sub_controller = n.subscribe("jarvis_controller", 1000, jarvis_controller_callback);
    ros::Subscriber sub_messenger = n.subscribe("jarvis_messenger", 1000, jarvis_messenger_callback);
    ros::Subscriber sub_voice = n.subscribe("jarvis_voice", 1000, jarvis_voice_callback);

    // keeps the node up and running
    ros::spin();
    return 0;
}