#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


int main(int argc, char **argv)
{
    // Inizialize a ROS node called talker
    ros::init(argc, argv, "simple_publisher_cpp");
    ros::NodeHandle n;

    // register a publisher on the topic /chatter that will publish String messages
    ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 10);

    // Define the frequency for publishing the messages
    // The rate is expressed in Hertz
    ros::Rate rate(10); // Hz


    int counter = 0;
    // Keep going publishing messages until the ROS communication is alive
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "Hello world " << counter;
        msg.data = ss.str();
        pub.publish(msg);

        ros::spinOnce();

        // wait the desired rate before publishing the next message
        rate.sleep();
        ++counter;
    }


  return 0;
}