#include "arduinobot_controller/arduinobot_hardware_interface.h"

ArduinobotHardwareInterface::ArduinobotHardwareInterface(ros::NodeHandle& nh) : nh_(nh)
{
    ROS_INFO("Starting Arduinobot Hardware Interface...");

    // connect and register joint state interface
    hardware_interface::JointStateHandle state_handle1("joint_1", &pos_[0], &vel_[0], &eff_[0]);
    joint_state_interface_.registerHandle(state_handle1);
    hardware_interface::JointStateHandle state_handle2("joint_2", &pos_[1], &vel_[1], &eff_[1]);
    joint_state_interface_.registerHandle(state_handle2);
    hardware_interface::JointStateHandle state_handle3("joint_3", &pos_[2], &vel_[2], &eff_[2]);
    joint_state_interface_.registerHandle(state_handle3);
    hardware_interface::JointStateHandle state_handle4("joint_4", &pos_[3], &vel_[3], &eff_[3]);
    joint_state_interface_.registerHandle(state_handle4);

    registerInterface(&joint_state_interface_);

    // connect and register joint position interface
    // the motors accept position inputs
    hardware_interface::JointHandle position_handle1(joint_state_interface_.getHandle("joint_1"), &cmd_[0]);
    joint_position_interface_.registerHandle(position_handle1);
    hardware_interface::JointHandle position_handle2(joint_state_interface_.getHandle("joint_2"), &cmd_[1]);
    joint_position_interface_.registerHandle(position_handle2);
    hardware_interface::JointHandle position_handle3(joint_state_interface_.getHandle("joint_3"), &cmd_[2]);
    joint_position_interface_.registerHandle(position_handle3);
    hardware_interface::JointHandle position_handle4(joint_state_interface_.getHandle("joint_4"), &cmd_[3]);
    joint_position_interface_.registerHandle(position_handle4);

    registerInterface(&joint_position_interface_);

    ROS_INFO("Interfaces registered.");


    ROS_INFO("Preparing the Controller Manager");

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    update_freq_ = ros::Duration(0.1);
    looper_ = nh_.createTimer(update_freq_, &ArduinobotHardwareInterface::update, this);
    
    ROS_INFO("Ready to execute the control loop");
}

void ArduinobotHardwareInterface::update(const ros::TimerEvent& e)
{
    ROS_INFO("Update Event");
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ArduinobotHardwareInterface::read()
{
    ROS_INFO("Read Event");
}

void ArduinobotHardwareInterface::write(ros::Duration elapsed_time)
{
    ROS_INFO("Write Event");
}


// TODO: move this in arduinobot_controller_node.cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "arduinobot_controller_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    ArduinobotHardwareInterface robot(nh);
    spinner.spin();
    return 0;
}