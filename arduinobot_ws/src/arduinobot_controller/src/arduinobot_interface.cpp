#include "arduinobot_controller/arduinobot_interface.h"

ArduinobotInterface::ArduinobotInterface(ros::NodeHandle& nh) : nh_(nh),
                pos_(4, 0),
                vel_(4, 0),
                eff_(4, 0),
                cmd_(4, 0)
{
    ROS_INFO("Starting Arduinobot Hardware Interface...");

    // connect and register joint state interface
    hardware_interface::JointStateHandle state_handle1("joint_1", &pos_.at(0), &vel_.at(0), &eff_.at(0));
    joint_state_interface_.registerHandle(state_handle1);
    hardware_interface::JointStateHandle state_handle2("joint_2", &pos_.at(1), &vel_.at(1), &eff_.at(1));
    joint_state_interface_.registerHandle(state_handle2);
    hardware_interface::JointStateHandle state_handle3("joint_3", &pos_.at(2), &vel_.at(2), &eff_.at(2));
    joint_state_interface_.registerHandle(state_handle3);
    hardware_interface::JointStateHandle state_handle4("joint_4", &pos_.at(3), &vel_.at(3), &eff_.at(3));
    joint_state_interface_.registerHandle(state_handle4);

    registerInterface(&joint_state_interface_);

    // connect and register joint position interface
    // the motors accept position inputs
    hardware_interface::JointHandle position_handle1(joint_state_interface_.getHandle("joint_1"), &cmd_.at(0));
    joint_position_interface_.registerHandle(position_handle1);
    hardware_interface::JointHandle position_handle2(joint_state_interface_.getHandle("joint_2"), &cmd_.at(1));
    joint_position_interface_.registerHandle(position_handle2);
    hardware_interface::JointHandle position_handle3(joint_state_interface_.getHandle("joint_3"), &cmd_.at(2));
    joint_position_interface_.registerHandle(position_handle3);
    hardware_interface::JointHandle position_handle4(joint_state_interface_.getHandle("joint_4"), &cmd_.at(3));
    joint_position_interface_.registerHandle(position_handle4);

    registerInterface(&joint_position_interface_);

    ROS_INFO("Interfaces registered.");


    ROS_INFO("Preparing the Controller Manager");

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    update_freq_ = ros::Duration(0.1);
    looper_ = nh_.createTimer(update_freq_, &ArduinobotInterface::update, this);
    
    ROS_INFO("Ready to execute the control loop");
}

void ArduinobotInterface::update(const ros::TimerEvent& e)
{
    ROS_INFO("Update Event");
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ArduinobotInterface::read()
{
    ROS_INFO("Read Event");
    ROS_INFO_STREAM("joint_1 position: " << pos_.at(0));
    ROS_INFO_STREAM("joint_2 position: " << pos_.at(1));
    ROS_INFO_STREAM("joint_3 position: " << pos_.at(2));
    ROS_INFO_STREAM("joint_4 position: " << pos_.at(3));
    pos_.at(0) = cmd_.at(0);
    pos_.at(1) = cmd_.at(1);
    pos_.at(2) = cmd_.at(2);
    pos_.at(3) = cmd_.at(3);
}

void ArduinobotInterface::write(ros::Duration elapsed_time)
{
    ROS_INFO("Write Event");
    ROS_INFO_STREAM("joint_1 position command: " << cmd_.at(0));
    ROS_INFO_STREAM("joint_2 position command: " << cmd_.at(1));
    ROS_INFO_STREAM("joint_3 position command: " << cmd_.at(2));
    ROS_INFO_STREAM("joint_4 position command: " << cmd_.at(3));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "arduinobot_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    ArduinobotInterface robot(nh);
    spinner.spin();
    return 0;
}