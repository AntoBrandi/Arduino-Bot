#include <ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

ros::NodeHandle  nh;


void callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    
}


ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> server("test_srv",&callback);


void setup()
{
  nh.initNode();
  nh.advertiseService(server);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
