/*
  arduinobot - ros_robot_control
  Script that creates a ROS node on the Arduino that subscribes to 
  joint angle messages and actuates the servo motors according to the
  selected angles
  
  Copyright (c) 2021 Antonio Brandi.  All right reserved.
*/

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

// Declare the Arduino pin where each servo is connected
#define SERVO_BASE_PIN 8
#define SERVO_SHOULDER_PIN 9
#define SERVO_ELBOW_PIN 10
#define SERVO_GRIPPER_PIN 11

// Define the working range for each joint
#define MIN_RANGE 0
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

// Initialize the ROS node
ros::NodeHandle  nh;

/*
 * This function moves a given servo smoothly from a given start position to a given end position.
 * The mouvement can be both clockwise or counterclockwise based on the values assigned to
 * the start position and end position
 */
void reach_goal(Servo& motor, int goal){
  if(goal>=motor.read()){
    // goes from the start point degrees to the end point degrees
    for (int pos = motor.read(); pos <= goal; pos += 1) { 
      motor.write(pos);     
      delay(5);                       
    }
  } else{
    // goes from the end point degrees to the start point degrees
    for (int pos = motor.read(); pos >= goal; pos -= 1) { 
      motor.write(pos);     
      delay(5);                       
    }
  }
}


/*
 * This function is called each time a new message is published on the topic /servo_actuate
 * The last message that was published on that topic is passed as input to this function.
 * The received message is converted to joint angles of each connected servo motor with the following 
 * order in the received array
 * 
 * base angle
 * shoulder angle
 * elbow angle
 * gripper angle
 * 
 * Then the received message is compared with fixed boundaries for each joint
 * and then each servo is actuated. 
 */
void arm_actuate_cb( const std_msgs::UInt16MultiArray& msg){

  // check that the received data are bounded correctly
  if(msg.data[0]<MIN_RANGE) msg.data[0] = MIN_RANGE;
  if(msg.data[1]<MIN_RANGE) msg.data[1] = MIN_RANGE;
  if(msg.data[2]<MIN_RANGE) msg.data[2] = MIN_RANGE;
  if(msg.data[3]<MIN_RANGE) msg.data[3] = MIN_RANGE;

  if(msg.data[0]>MAX_RANGE) msg.data[0] = MAX_RANGE;
  if(msg.data[1]>MAX_RANGE) msg.data[1] = MAX_RANGE;
  if(msg.data[2]>MAX_RANGE) msg.data[2] = MAX_RANGE;
  if(msg.data[3]>MAX_RANGE) msg.data[3] = MAX_RANGE;

  reach_goal(base, msg.data[0]);
  reach_goal(shoulder, msg.data[1]);
  reach_goal(elbow, msg.data[2]);
  reach_goal(gripper, msg.data[3]);
}

// Define the subscriber to the topic /servo_actuate where are published UInt16MultiArray messages
// Define the function that will be triggered each time a new message is published on this topic
ros::Subscriber<std_msgs::UInt16MultiArray> sub("arduino/arm_actuate", &arm_actuate_cb );


void setup() {
  // Attach and Initialize each Servo to the Arduino pin where it is connected
  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  gripper.attach(SERVO_GRIPPER_PIN); 

  // Set a common start point for each joint
  // This way, the start status of each joint is known
  base.write(BASE_START);
  shoulder.write(SHOULDER_START);
  elbow.write(ELBOW_START);
  gripper.write(GRIPPER_START);

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
