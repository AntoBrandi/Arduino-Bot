/*
  arduinobot - arduino_simple_subscriber
  Simple Arduino ROS subscriber.
  This script creates a ROS node on the Arduino that subscribe to a ROS topic 
  and turns on/off a led when a new message is received

  Copyright (c) 2021 Antonio Brandi.  All right reserved.
*/

#include <ros.h>
#include <std_msgs/Empty.h>

#define LED_PIN 12

// Define a ROS Node on the Arduino
ros::NodeHandle  nh;

// This function is called when a new message is published on the topic /servo_actuate
void messageCb( const std_msgs::Empty& msg){
  digitalWrite(LED_PIN, HIGH-digitalRead(LED_PIN));   // turn on/off the led
}

// Define the subscriber to the topic /servo_actuate where are published empty messages
// Define the function that will be triggered each time a new message is published on this topic
ros::Subscriber<std_msgs::Empty> sub("servo_actuate", &messageCb );

void setup()
{ 
  // init the LED
  pinMode(LED_PIN, OUTPUT);
  // Inizialize the ROS node on the Arduino
  nh.initNode();
  // Inform ROS that this node will subscribe to messages on a given topic
  nh.subscribe(sub);
}

void loop()
{  
  // Keep the ROS node up and running
  nh.spinOnce();
  delay(1);
}
