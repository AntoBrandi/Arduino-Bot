#include "ros/ros.h"
#include "std_msgs/String.h"


// This function is called each time a new message is published on the topic /chatter
// The message that has been published is then passed as input to this function
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    // Inizialize a ROS node called listener
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    // register a subscriber on the topic /chatter that will listen for String messages
    // when a new message is received, the callback function is triggered and starts its execution
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    // keeps the node up and running
    ros::spin();
    return 0;
}