#include "ros/ros.h"


void timerCallback(const ros::TimerEvent& event)
{
    // Function that will be called at each expiration of the timer
    ROS_INFO("Called timerCallback Function");
}


int main(int argc, char **argv)
{
    // Inizialize a ROS node called timer_node
    ros::init(argc, argv, "timer_node");
    ros::NodeHandle nh;

    // Set the duration of the timer
    ros::Duration timer_duration(1);

    // Create an instance of the timer with its duration and the function that
    // will be called at each time duration
    ros::Timer timer = nh.createTimer(timer_duration, timerCallback);

    // Keep the node up and running
    ros::spin();
    return 0;
}