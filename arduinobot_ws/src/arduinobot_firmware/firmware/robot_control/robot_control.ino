#include <Servo.h>

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

  // Start the Serial communication with ROS
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available())
  {
    String msg = Serial.readStringUntil('\n');
    uint8_t j = 0;
    uint8_t last_idx = 0;
    uint8_t joints[4];
    for(uint8_t i = 0; i < msg.length(); i++)
    {
      if (msg.charAt(i) == ',')
      {
        joints[j] = msg.substring(last_idx, i).toInt();
        j++;
        last_idx = i + 1;
      }
    }
    // Add last element
    joints[j] = msg.substring(last_idx, msg.length()).toInt();
    
    // check that the received data are bounded correctly
    if(joints[0]<MIN_RANGE) joints[0] = MIN_RANGE;
    if(joints[1]<MIN_RANGE) joints[1] = MIN_RANGE;
    if(joints[2]<MIN_RANGE) joints[2] = MIN_RANGE;
    if(joints[3]<MIN_RANGE) joints[3] = MIN_RANGE;
  
    if(joints[0]>MAX_RANGE) joints[0] = MAX_RANGE;
    if(joints[1]>MAX_RANGE) joints[1] = MAX_RANGE;
    if(joints[2]>MAX_RANGE) joints[2] = MAX_RANGE;
    if(joints[3]>MAX_RANGE) joints[3] = MAX_RANGE;
  
    reach_goal(base, joints[0]);
    reach_goal(shoulder, joints[1]);
    reach_goal(elbow, joints[2]);
    reach_goal(gripper, joints[3]);
  }
  delay(0.01);
}
