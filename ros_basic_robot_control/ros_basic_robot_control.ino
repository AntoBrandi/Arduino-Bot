/*
  arduinobot
  Script that creates a ROS node on the Arduino that subscribes to topic
  for the joint contorl of the arm and the one of the gripper.
  When a new messages is published on a topic, th 
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int16.h>

// Declare the Arduino pin where each servo is connected
#define SERVO_BASE_PIN 8
#define SERVO_SHOULDER_PIN 9
#define SERVO_ELBOW_PIN 10
#define SERVO_GRIPPER_PIN 11

// Define the working range for each joint
#define MIN_RANGE 0
#define MIN_RANGE_GRIPPER 40
#define MAX_RANGE 180

// Register the servo motors of each joint
Servo base;  
Servo shoulder;  
Servo elbow;  
Servo gripper;  

ros::NodeHandle  nh;

// Variable that keeps track of the current position of each joint
int last_angle_base = 0;
int last_angle_shoulder = 0;
int last_angle_elbow = 0;
int last_angle_gripper = 0;

/*
 * This function moves a given servo smoothly from a given start position to a given end position.
 * The mouvement can be both clockwise or counterclockwise based on the values assigned to
 * the start position and end position
 */
void reach_goal(Servo servo, int start_point, int end_point){
  if(end_point>=start_point){
    for (int pos = start_point; pos <= end_point; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);     
    delay(5);                       
    }
  } else{
    for (int pos = start_point; pos >= end_point; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);     
    delay(5);                       
    }
  }
}


void moveArm(int base_angle, int shoulder_angle, int elbow_angle, int gripper_angle){
  reach_goal(base, last_angle_base, base_angle);
  reach_goal(shoulder, last_angle_shoulder, shoulder_angle);
  reach_goal(elbow, last_angle_elbow, elbow_angle);
  reach_goal(gripper, last_angle_gripper, gripper_angle);
  last_angle_base = base_angle;
  last_angle_shoulder = shoulder_angle;
  last_angle_elbow = elbow_angle;
  last_angle_gripper = gripper_angle;
}


void servoActuateCb( const std_msgs::UInt8MultiArray& msg){
  int base_angle = (int)msg.data[0];
  int shoulder_angle = (int)msg.data[1];
  int elbow_angle = (int)msg.data[2];
  int gripper_angle = (int)msg.data[3];

  // check that the received data are bounded correctly
  if(base_angle<MIN_RANGE) base_angle = MIN_RANGE;
  if(shoulder_angle<MIN_RANGE) shoulder_angle = MIN_RANGE;
  if(elbow_angle<MIN_RANGE) elbow_angle = MIN_RANGE;
  if(gripper_angle<MIN_RANGE_GRIPPER) gripper_angle = MIN_RANGE_GRIPPER;

  if(base_angle>MAX_RANGE) base_angle = MAX_RANGE;
  if(shoulder_angle>MAX_RANGE) shoulder_angle = MAX_RANGE;
  if(elbow_angle>MAX_RANGE) elbow_angle = MAX_RANGE;
  if(gripper_angle>MAX_RANGE) gripper_angle = MAX_RANGE;

  moveArm(base_angle, shoulder_angle, elbow_angle, gripper_angle);
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub("servo_actuate", &servoActuateCb );


void setup() {
  // Attach each Servo to the Arduino pin where it is connected
  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  gripper.attach(SERVO_GRIPPER_PIN); 

  // Set a common start point for each joint
  base.write(0);
  shoulder.write(0);
  elbow.write(0);
  gripper.write(0);

   // Inizialize the ROS node on the Arduino
  nh.initNode();
  // Inform ROS that this node will subscribe to messages on a given topic
  nh.subscribe(sub);
}

void loop() {
  // Keep the ROS node up and running
  nh.spinOnce();
  delay(1);
}
