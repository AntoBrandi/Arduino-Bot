/*
  arduinobot
  Simple Arduino ROS publisher.
  This script creates a ROS node on the Arduino that publish a string message each 1000ms
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include <ros.h>
#include <std_msgs/String.h>

// Define a ROS Node on the Arduino
ros::NodeHandle  nh;

// Define the type of the message that will be published 
std_msgs::String msg;
// Define the topic name on which the message will be published
ros::Publisher pub("servo_control", &msg);
// Constant message that will be published on this topic
char hello[13] = "hello world!";

void setup()
{
  // Inizialize the ROS node on the Arduino
  nh.initNode();
  // Inform ROS that this node will publish messages on a given topic
  nh.advertise(pub);
}

void loop()
{
  // Compose the ROS message and publish it
  msg.data = hello;
  pub.publish(&msg);

  // Keep the ROS node up and running
  nh.spinOnce();
  // Wait before publish the next message
  delay(1000);
}
