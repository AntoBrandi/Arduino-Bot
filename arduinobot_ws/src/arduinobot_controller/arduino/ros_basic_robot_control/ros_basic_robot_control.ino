/*
  arduinobot - ros_basic_robot_control
  Script that creates a ROS node on the Arduino that subscribes to 
  joint angle messages and actuates the servo motors according to the
  selected angles
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
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
#define MIN_RANGE_GRIPPER 40
#define MAX_RANGE 180

// Register the servo motors of each joint
Servo base;  
Servo shoulder;  
Servo elbow;  
Servo gripper;  

// Initialize the ROS node
ros::NodeHandle  nh;

// Keep track of the last angle of each servo
// When a new angle is assigned to a servo, start its movement from 
// the last known position instead of restarting from 0
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
    // goes from the start point degrees to the end point degrees
    for (int pos = start_point; pos <= end_point; pos += 1) { 
    servo.write(pos);     
    delay(5);                       
    }
  } else{
    // goes from the end point degrees to the start point degrees
    for (int pos = start_point; pos >= end_point; pos -= 1) { 
    servo.write(pos);     
    delay(5);                       
    }
  }
}

/*
 * This function triggers the actuation of each servo motor of the arm 
 * and keeps track of the last position of each joint for the next execution.
 * So that, the start point for the current actuation of the servo motors is the 
 * last registered position for the joint
 */
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
void servoActuateCb( const std_msgs::UInt16MultiArray& msg){
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

// Define the subscriber to the topic /servo_actuate where are published UInt16MultiArray messages
// Define the function that will be triggered each time a new message is published on this topic
ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo_actuate", &servoActuateCb );


void setup() {
  // Attach and Initialize each Servo to the Arduino pin where it is connected
  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  gripper.attach(SERVO_GRIPPER_PIN); 

  // Set a common start point for each joint
  // This way, the start status of each joint is known
  base.write(90);
  shoulder.write(90);
  elbow.write(90);
  gripper.write(90);

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
