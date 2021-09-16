#pragma once

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

class ArduinobotHardwareInterface: public hardware_interface::RobotHW {

    public:

        ArduinobotHardwareInterface(ros::NodeHandle&);

        void update(const ros::TimerEvent& e);    
        void read();
        void write(ros::Duration elapsed_time);
    
    private:

        ros::NodeHandle nh_;
        ros::Duration elapsed_time_;
        ros::Duration update_freq_;
        ros::Timer looper_;

        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface joint_position_interface_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
        
        double cmd_[4];
        double pos_[4];
        double vel_[4];
        double eff_[4];

};
