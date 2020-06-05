#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

#define JOINT0 8
#define JOINT1 9
#define JOINT2 10
#define MIN_RANGE 0
#define MAX_RANGE 180
#define PUBLISH_RATE 10 //Hz

Servo servo_joint0;
Servo servo_joint1;
Servo servo_joint2;

ros::NodeHandle nh;
std_msgs::UInt16MultiArray actual_position;
ros::Publisher pub("servo_position", &actual_position);

int counter;

void actuate(const std_msgs::UInt16MultiArray& msg){
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


void publish(){
  actual_position.data[0] = servo_joint0.read();
  actual_position.data[1] = servo_joint1.read();
  actual_position.data[2] = servo_joint2.read();
  pub.publish(&actual_position);
}



ros::Subscriber<std_msgs::UInt16MultiArray> sub("/servo_actuator", actuate );



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
  // setup for the publisher
  actual_position.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  actual_position.layout.dim[0].label = "joint";
  actual_position.layout.dim[0].size = 4;
  actual_position.layout.dim[0].stride = 1;
  actual_position.layout.data_offset = 0;
  actual_position.data = (int *)malloc(sizeof(int)*8);
  actual_position.data_length = 3;
  nh.advertise(pub);
  // inizialize th counter for the delay in the publishing procedure
  counter = 0;
}

void loop()
{
  // make the publish rate independent from the subscribe rate
  if(counter>=1000/PUBLISH_RATE){
    publish();
    // reset the counter
    counter = 0;
  }
  nh.spinOnce();
  counter+=1;
  delay(1); 
}
