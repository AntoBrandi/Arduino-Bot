/*
  arduinobot - moveit_interface

  This script uses the MoveIt! API for C++ in order to reach a given goal
  and eventually prints the current robot status and informations.

  When triggered, the reach_goal function uses the MoveIt! API for creating a planning request
  that generates a new trajectory that allows the robot to reach its new destination.
  Then, this trajectory is executed via controllers that have been registered and launched with MoveIt!

  Copyright (c) 2021 Antonio Brandi.  All right reserved.
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>


const std::string ARM_GROUP_NAME = "arduinobot_arm";
const std::string GRIPPER_GROUP_NAME = "arduinobot_hand";

class MoveitInterface
{
private:
    moveit::planning_interface::MoveGroupInterface arm_move_group_;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_;


public:
    MoveitInterface() : arm_move_group_(ARM_GROUP_NAME),
                        gripper_move_group_(GRIPPER_GROUP_NAME)
    {}

    void reach_goal(std::vector<double> arm_goal, std::vector<double> gripper_goal)
    {
      arm_move_group_.setJointValueTarget(arm_goal);
      gripper_move_group_.setJointValueTarget(gripper_goal);

      // blocking functions below, will return after the execution
      arm_move_group_.move();
      gripper_move_group_.move();

      // Make sure that no residual movement remains
      arm_move_group_.stop();
      gripper_move_group_.stop();
    }

    void set_max_velocity(double scaling_factor)
    {
      arm_move_group_.setMaxVelocityScalingFactor(scaling_factor);
      gripper_move_group_.setMaxVelocityScalingFactor(scaling_factor);
    }

    void set_max_acceleration(double scaling_factor)
    {
      arm_move_group_.setMaxAccelerationScalingFactor(scaling_factor);
      gripper_move_group_.setMaxAccelerationScalingFactor(scaling_factor);
    }

};