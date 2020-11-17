/*
  arduinobot
  Script that enables the control of the robot arm via terminal.
  It enables to control each servo at each joint by indicating it's angle.
  Run it with:
    - Load the script on the Arduino
    - Open the Serial monitor setting 9600 baud
    - write the number of the joint to move (1 to 4)
    - Write the angle of the selected joint (1 to 180)
  
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include <Servo.h>

// Declare the Arduino pin where each servo is connected
#define SERVO_BASE_PIN 8
#define SERVO_SHOULDER_PIN 9
#define SERVO_ELBOW_PIN 10
#define SERVO_GRIPPER_PIN 11

// Register the servo motors of each joint
Servo base;  
Servo shoulder;  
Servo elbow;  
Servo gripper;  
 
String input_joint = "";
String input_angle = "";  
int angle = 0;
int joint = 0;
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
    delay(15);                       
    }
  } else{
    for (int pos = start_point; pos >= end_point; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);     
    delay(15);                       
    }
  }
}

/*
 * This function checks the inputs of the user and associates at each input the 
 * respective servo motor and triggers its mouvement.
 * Finally, it updates the variable that keeps track of the curren position of each servo
 * that is used as start point for the next mouvement
 */
void move(int joint, int target){
  switch(joint){
    case 1: // if (joint==1)
      Serial.println("Moving base");
      reach_goal(base, last_angle_base, target);
      last_angle_base = target;
      break;
    case 2: // else if (joint==1)
      Serial.println("Moving shoulder");
      reach_goal(shoulder, last_angle_shoulder, target);
      last_angle_shoulder = target;
      break;
    case 3:
      Serial.println("Moving elbow");
      reach_goal(elbow, last_angle_elbow, target);
      last_angle_elbow = target;
      break;
    case 4:
      Serial.println("Moving gripper");
      reach_goal(gripper, last_angle_gripper, target);
      last_angle_gripper = target;
      break;
    default:
      break;    
  }
}

void setup() {
  // Enable the serial communication for displaying information and reading user inputs
  Serial.begin(9600);
  
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
}

void loop() {
  // Wait for user input
  // First insert the joint number (1 to 4) and press ENTER
  // Then insert the angle number (1 to 180) and press ENTER
  Serial.println("Insert the joint number (1-4)");
  if(Serial.available()>0){
    if(input_joint.equals("")){
      input_joint = Serial.readString();
    } 
    else
    {
      joint = input_joint.toInt();
      Serial.println("Insert the angle number (1-180)");
      input_angle = Serial.readString();
      if(!input_angle.equals("")){
        angle = input_angle.toInt();
        input_joint = "";
        input_angle = "";
      }
    }
  }

  // If valid input have been inserted by the user,
  // Then trigger a mouvement of the selected joint of the selected angle
  if(joint>0 && joint<5 && angle>0 && angle<181){
    Serial.println("Moving");
    Serial.print("Angle: ");
    Serial.println(String(angle));
    Serial.print("Joint: ");
    Serial.println(String(joint));
    move(joint, angle);
    joint = -1;
    angle = -1;
  }
}
