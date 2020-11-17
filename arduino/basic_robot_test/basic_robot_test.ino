/*
  arduinobot - basic_robot_test
  Script that enables some test mouvements of the robot arm 
  by controlling all the servo motors smoothly in their complete range.
  All the servo motors are moved sinconously with the same angle
  that autoincrements or decrements
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include <Servo.h>

// Declare the Arduino pin where each servo is connected
#define SERVO_BASE_PIN 8
#define SERVO_SHOULDER_PIN 9
#define SERVO_ELBOW_PIN 10
#define SERVO_GRIPPER_PIN 11

// Create an Instance of the servo motors for each joint
Servo base;  
Servo shoulder;  
Servo elbow;  
Servo gripper;  

// Variable that keeps track of the current position of each joint
// the current position will be the same for all the joints
int pos = 0;    

void setup() {
  // Attach each Servo to the Arduino pin where it is connected
  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  gripper.attach(SERVO_GRIPPER_PIN);  
}

void loop() {
  // Move sincronously all the servo motors
  // in the first movement from 0 to 180 degrees and in
  // the second movement from 180 to 0 degrees
  // First Movement
  // Increment each Servo angle by 1 degree each 15 ms
  for (pos = 0; pos <= 180; pos += 1) { 
    // in steps of 1 degree
    base.write(pos);     
    shoulder.write(pos);  
    elbow.write(pos);
    gripper.write(pos);
    delay(15);                       
  }
  // Second Movement
  // Decrement each Servo angle by 1 degree each 15 ms
  for (pos = 180; pos >= 0; pos -= 1) { 
    base.write(pos);              
    shoulder.write(pos);
    elbow.write(pos);
    gripper.write(pos);
    delay(15);                      
  }
}
