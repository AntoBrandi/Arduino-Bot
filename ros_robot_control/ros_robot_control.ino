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
#include <std_msgs/UInt8.h>


// Declare the Arduino pin where each servo is connected
#define SERVO_BASE_PIN 8
#define SERVO_SHOULDER_PIN 9
#define SERVO_ELBOW_PIN 10
#define SERVO_GRIPPER_PIN 11

// Define the working range for each joint
#define MIN_RANGE 0
#define MIN_RANGE_GRIPPER 40
#define MAX_RANGE 180

// Define the start configuration of the joints
#define BASE_START 90
#define SHOULDER_START 90
#define ELBOW_START 90
#define GRIPPER_START 0

// Register the servo motors of each joint
Servo base;  
Servo shoulder;  
Servo elbow;  
Servo gripper;  

ros::NodeHandle  nh;

// Variable that keeps track of the current position of each joint
int last_angle_base = BASE_START;
int last_angle_shoulder = SHOULDER_START;
int last_angle_elbow = ELBOW_START;
int last_angle_gripper = GRIPPER_START;

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

void move_arm(int base_angle, int shoulder_angle, int elbow_angle){
  reach_goal(base, last_angle_base, base_angle);
  reach_goal(shoulder, last_angle_shoulder, shoulder_angle);
  reach_goal(elbow, last_angle_elbow, elbow_angle);
  last_angle_base = base_angle;
  last_angle_shoulder = shoulder_angle;
  last_angle_elbow = elbow_angle;
}

void move_gripper(int gripper_angle){
  reach_goal(gripper, last_angle_gripper, gripper_angle);
  last_angle_gripper = gripper_angle;
}

void arm_actuate_cb( const std_msgs::UInt8MultiArray& msg){
  int base_angle = (int)msg.data[0];
  int shoulder_angle = (int)msg.data[1];
  int elbow_angle = (int)msg.data[2];

  // check that the received data are bounded correctly
  if(base_angle<MIN_RANGE) base_angle = MIN_RANGE;
  if(shoulder_angle<MIN_RANGE) shoulder_angle = MIN_RANGE;
  if(elbow_angle<MIN_RANGE) elbow_angle = MIN_RANGE;

  if(base_angle>MAX_RANGE) base_angle = MAX_RANGE;
  if(shoulder_angle>MAX_RANGE) shoulder_angle = MAX_RANGE;
  if(elbow_angle>MAX_RANGE) elbow_angle = MAX_RANGE;

  move_arm(base_angle, shoulder_angle, elbow_angle);
}

void gripper_actuate_cb(const std_msgs::UInt8& msg){
  int gripper_angle = (int)msg.data;

  if(gripper_angle<MIN_RANGE_GRIPPER) gripper_angle = MIN_RANGE_GRIPPER;
  if(gripper_angle>MAX_RANGE) gripper_angle = MAX_RANGE;

  move_gripper(gripper_angle);
}

// Subscribers
ros::Subscriber<std_msgs::UInt8MultiArray> sub_arm("arduinobot_arm_controller/arduino/arm_actuate", &arm_actuate_cb );
ros::Subscriber<std_msgs::UInt8> sub_gripper("arduinobot_gripper_controller/arduino/gripper_actuate", &gripper_actuate_cb );


void setup() {
  // Attach each Servo to the Arduino pin where it is connected
  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  gripper.attach(SERVO_GRIPPER_PIN); 

  // Set a common start point for each joint
  base.write(BASE_START);
  shoulder.write(SHOULDER_START);
  elbow.write(ELBOW_START);
  gripper.write(GRIPPER_START);

  // Inizialize the ROS node on the Arduino
  nh.initNode();
  // Inform ROS that this node will subscribe to messages on a given topic
  nh.subscribe(sub_arm);
  nh.subscribe(sub_gripper);
}

void loop() {
  // Keep the ROS node up and running
  nh.spinOnce();
  delay(1);
}
