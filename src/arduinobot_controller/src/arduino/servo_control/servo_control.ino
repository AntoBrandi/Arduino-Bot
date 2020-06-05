#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>

#define JOINT0 8
#define JOINT1 9
#define JOINT2 10
#define MIN_RANGE 0
#define MAX_RANGE 180

Servo servo_joint0;
Servo servo_joint1;
Servo servo_joint2;

void actuate(const std_msgs::UInt8MultiArray& msg){
  // check that the received data are bounded correctly
  if(msg.data[0]<MIN_RANGE) msg.data[0] = MIN_RANGE;
  if(msg.data[1]<MIN_RANGE) msg.data[1] = MIN_RANGE;
  if(msg.data[2]<MIN_RANGE) msg.data[2] = MIN_RANGE;

  if(msg.data[0]>MAX_RANGE) msg.data[0] = MAX_RANGE;
  if(msg.data[1]>MAX_RANGE) msg.data[1] = MAX_RANGE;
  if(msg.data[2]>MAX_RANGE) msg.data[2] = MAX_RANGE;

  // apply the received angles to the servo motors
  servo_joint0.write((int)msg.data[0]);
  servo_joint1.write((int)msg.data[1]);
  servo_joint2.write((int)msg.data[2]);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt8MultiArray> sub("/servo_actuator", actuate );


void setup()
{
  // set the attached pin of each joint
  servo_joint0.attach(JOINT0);
  servo_joint1.attach(JOINT1);
  servo_joint2.attach(JOINT2);
  // set an initial value for all the joints
  servo_joint0.write(90);
  servo_joint1.write(90);
  servo_joint2.write(90);
  // cerate and initialize a ROS node that subscibes the topic /servo_actuator
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1); 
}
