/*
  arduinobot
  Simple Arduino ROS subscriber.
  This script creates a ROS node on the Arduino that subscribe to a ROS topic 
  and turns on/off a led when a new message is received
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include <ros.h>
#include <std_msgs/Empty.h>

#define LED_PIN 12

// Define a ROS Node on the Arduino
ros::NodeHandle  nh;

void messageCb( const std_msgs::Empty& msg){
  digitalWrite(LED_PIN, HIGH-digitalRead(LED_PIN));   // blink the led
}

// Define the topic name on which it is subscribed
ros::Subscriber<std_msgs::Empty> sub("servo_actuate", &messageCb );

void setup()
{ 
  // register the LED
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
