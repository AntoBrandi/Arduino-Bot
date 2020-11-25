/*
  arduinobot - angles_converter

  This script implements two services on the topics
    - radians_to_degrees
    - degrees_to_radians

  Both of them receives a request with the format:
    float64 base
    float64 shoulder
    float64 elbow 
    float64 gripper
    
  and sends a response in the same format to the client

  The first service (radians_to_degrees) receives the angles in radians and convert
  those in degrees according to the boundaries defined inthe URDF file

  The second service (degrees_to_radians) receives the angles in degrees and convert
  those in radians according to the boundaries defined inthe URDF file

  This conversion is needed for the control of the real robot in order to convert the radians angle of each joint
  as used in ROS in degrees angles as used in Arduino for the actuation of the Servo motors
  

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include "ros/ros.h"
#include "arduinobot_controller/AnglesConverter.h"
#include "math.h"


bool convert_radians_to_degrees(arduinobot_controller::AnglesConverter::Request  &req,
         arduinobot_controller::AnglesConverter::Response &res)
{
    // Function that is called every time the service radians_to_degrees is called
    // It receives the Request message as input with the angles in radians and the Result message Type as input 
    // it fills out the response message with the converted values 
    // and returns the Result message as output with the angles in degrees
    res.base = static_cast<int>(((req.base+(M_PI/2))*180)/M_PI);
    res.shoulder = 180-static_cast<int>(((req.shoulder+(M_PI/2))*180)/M_PI);
    res.elbow = static_cast<int>(((req.elbow+(M_PI/2))*180)/M_PI);
    res.gripper = static_cast<int>(((-req.gripper)*180)/(M_PI/2));
    return true;
}

bool convert_degrees_to_radians(arduinobot_controller::AnglesConverter::Request  &req,
         arduinobot_controller::AnglesConverter::Response &res)
{
    // Function that is called every time the service radians_to_degrees is called
    // It receives the Request message with the angles in degrees and the Result message Type as input 
    // it fills out the response message with the converted values 
    // and returns the Result message as output with the angles in radians
    res.base = ((M_PI*req.base) - ((M_PI/2)*180))/180;
    res.shoulder = (((180-req.shoulder)*M_PI)-((M_PI/2)*180))/180;
    res.elbow = ((M_PI*req.elbow) - ((M_PI/2)*180))/180;
    res.gripper = -((M_PI/2)*req.gripper)/180;
    return true;
}

int main(int argc, char **argv)
{
    // Inizialize a ROS node called angles_converter
    ros::init(argc, argv, "angles_converter");
    ros::NodeHandle n;

    // Inizialize two services for the angle conversions 
    ros::ServiceServer radians_to_degrees = n.advertiseService("radians_to_degrees", convert_radians_to_degrees);
    ros::ServiceServer degrees_to_radians = n.advertiseService("degrees_to_radians", convert_degrees_to_radians);
    
    ROS_INFO("Angles Converter Service Started");

    // keeps the node up and running
    ros::spin();
    return 0;
}