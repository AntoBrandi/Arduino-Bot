#include "ros/ros.h"
#include "arduinobot_test/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    // When this script is launched, get the args that are passed
    // to the script when is launched. This script requires 2 args from the user
    // that are the two numbers that will be added
    ros::init(argc, argv, "add_two_ints_client");
    if (argc != 3)
    {
        ROS_INFO("Requested two arguments");
        return 1;
    }


    // Sum the two numbers passed as arguments to this script 
    // by calling the action server that adds two integers
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<arduinobot_test::AddTwoInts>("add_two_ints");
    arduinobot_test::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);

    // Show the response of the service
    ROS_INFO("Requesting %d + %d", (int)srv.request.a, (int)srv.request.b);
    if (client.call(srv))
    {
        ROS_INFO("Service Response %d", (int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }

    return 0;
}