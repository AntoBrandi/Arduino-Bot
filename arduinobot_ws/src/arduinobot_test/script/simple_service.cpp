#include "ros/ros.h"
#include "arduinobot_test/AddTwoInts.h"


bool add(arduinobot_test::AddTwoInts::Request  &req,
         arduinobot_test::AddTwoInts::Response &res)
{
    // Function that is called each time the service receives a request
    // The request message is passed as input to this function

    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

int main(int argc, char **argv)
{
    // Inizialize a ROS node called simple_service
    ros::init(argc, argv, "simple_service");
    ros::NodeHandle n;

    // Initialize a service named add_two_ints that uses the AddTwoInts
    // for the Request/Response communication interface
    // Define the function that is called each time the service receives a request
    ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");

    // keeps the node up and running
    ros::spin();
    return 0;
}