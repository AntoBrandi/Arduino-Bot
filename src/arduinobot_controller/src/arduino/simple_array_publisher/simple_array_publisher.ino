#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>


#define PUBLISH_RATE 10 //Hz


ros::NodeHandle nh;
std_msgs::UInt16MultiArray actual_position;
ros::Publisher pub("servo_position", &actual_position);


int counter;


void publish(){
  
  actual_position.data[0] = 100;
  actual_position.data[1] = 0;
  actual_position.data[2] = 90;
  pub.publish(&actual_position);
}


void setup()
{
  // cerate and initialize a ROS node that subscibes the topic /servo_actuator
  nh.initNode();
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
