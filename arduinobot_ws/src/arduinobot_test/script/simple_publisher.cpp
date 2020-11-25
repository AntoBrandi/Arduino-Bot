#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


int main(int argc, char **argv)
{
    // Inizialize a ROS node called talker
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    // register a publisher on the topic /chatter that will publish String messages
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // Define the frequency for publishing the messages
    // The rate is expressed in Hertz
    ros::Rate loop_rate(10); // Hz


    int count = 0;
    // Keep going publishing messages until the ROS communication is alive
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        // wait the desired rate before publishing the next message
        loop_rate.sleep();
        ++count;
    }


  return 0;
}