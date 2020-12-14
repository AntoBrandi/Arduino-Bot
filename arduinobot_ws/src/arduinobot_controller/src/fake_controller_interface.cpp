/*
  arduinobot - fake_controller_interface
  
  This script implements a broker between the gazebo joint position controller taht actuates the simulated robot 
  via Float64 messages on the topics /arduinobot_sim/joint_**joint_number**_position_controller/command
  and the FollowJointTrajectoryAction and GripperCommandAction trajectory controllers that 
  implements an Action Server in order to actuate the arm and the gripper with a give trajectory

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"


ros::Publisher pub_joint_1;
ros::Publisher pub_joint_2;
ros::Publisher pub_joint_3;
ros::Publisher pub_joint_4;
ros::Publisher pub_joint_5;

void arm_actuate_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // Callback function that is called each time a new message is published on the topic /arduinobot_arm_controller/arduino_sim/arm_actuate
  // the received message is passed as input to this function
  // It uses the received message to actuate the arm joints of the simulated robot
  std_msgs::Float64 base;
  std_msgs::Float64 shoulder;
  std_msgs::Float64 elbow;

  base.data = msg->data[0];
  shoulder.data = msg->data[1];
  elbow.data = msg->data[2];

  pub_joint_1.publish(base);
  pub_joint_2.publish(shoulder);
  pub_joint_3.publish(elbow);
}

void gripper_actuate_cb(const std_msgs::Float64::ConstPtr& msg)
{
  // Callback function that is called each time a new message is published on the topic arduinobot_gripper_controller/arduino_sim/gripper_actuate
  // the received message is passed as input to this function
  // It uses the received message to actuate the gripper joints of the simulated robot
  std_msgs::Float64 gripper_left;
  std_msgs::Float64 gripper_right;

  gripper_left.data = msg->data;
  gripper_right.data = -msg->data;

  pub_joint_4.publish(gripper_left);
  pub_joint_5.publish(gripper_right);
}

int main(int argc, char **argv)
{
    // Inizialize a ROS node called listener
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    // register the publishers for the position controller topics that are publishing Float64 messages
    // which indicates the angle in radians that a given joint will be actuated
    pub_joint_1 = n.advertise<std_msgs::Float64>("/arduinobot_sim/joint_1_position_controller/command", 1000);
    pub_joint_2 = n.advertise<std_msgs::Float64>("/arduinobot_sim/joint_2_position_controller/command", 1000);
    pub_joint_3 = n.advertise<std_msgs::Float64>("/arduinobot_sim/joint_3_position_controller/command", 1000);
    pub_joint_4 = n.advertise<std_msgs::Float64>("/arduinobot_sim/joint_4_position_controller/command", 1000);
    pub_joint_5 = n.advertise<std_msgs::Float64>("/arduinobot_sim/joint_5_position_controller/command", 1000);

    // register a subscriber on the topic /arduinobot_arm_controller/arduino_sim/arm_actuate and on tthe topic 
    // arduinobot_gripper_controller/arduino_sim/gripper_actuate that will listen for Float64MultiArray messages indicating the 
    // desidered joint angles for the arm joint and will listen for Float64 messages indicating the desidered joint angle
    // for the gripper joint
    // when a new message is received, the callback function is triggered and starts its execution
    ros::Subscriber sub_arm = n.subscribe("arduinobot_arm_controller/arduino_sim/arm_actuate", 1000, arm_actuate_cb);
    ros::Subscriber sub_gripper = n.subscribe("arduinobot_gripper_controller/arduino_sim/gripper_actuate", 1000, gripper_actuate_cb);

    // keeps the node up and running
    ros::spin();
    return 0;
}