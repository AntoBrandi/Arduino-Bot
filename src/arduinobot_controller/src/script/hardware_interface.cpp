#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/UInt16MultiArray.h>
#include <ros/ros.h>

class ArduinoBot : public hardware_interface::RobotHW
{
public:
  ArduinoBot() 
 { 
   // init the publisher and subscriber node 
   ros::init(argc, argv, "node_interface");
   pub = n.advertise<std_msgs::UInt16MultiArray>("servo_actuator",1000);
   sub = n.suscribe("servo_position",1000,read);
   msg.data.clear();

   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_1("joint_1", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_1);

   hardware_interface::JointStateHandle state_handle_2("joint_2", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_2);

   hardware_interface::JointStateHandle state_handle_3("joint_3", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_3);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("joint_1"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_1);

   hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("joint_2"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_2);

   hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("joint_3"), &cmd[2]);
   jnt_pos_interface.registerHandle(pos_handle_3);

   registerInterface(&jnt_pos_interface);
  }

  read(const std_msgs::UInt16MultiArray& joints)
  {
      // Asks data about the servo motor position to the arduino controller
      for(int i = 0; i<3;i++){
        pos[i]=joints.data[i];
      }
  }

  write()
  {
      // Sends data about servo motor position to the arduino controller
      for (int i = 0; i<3;i++){
        msg.data.push_back((int)((cmd[i]+1.57075)*180)/3.1415);
      }
      pub.publish(msg);
  }

  sleep()
  {
    rate.sleep();
  }


private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Rate rate(10);
  std_msgs::UInt16MultiArray msg;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  // input from the controllers
  double cmd[3];
  // output of the robot
  double pos[3];
  double vel[3];
  double eff[3];
};


int main(int argc, char **argv)
{
  ArduinoBot robot;
  controller_manager::ControllerManager cm(&robot);


  while (true)
  {
     cm.update(robot.get_time(), robot.get_period());
     robot.write();
     robot.sleep();
  }

  return 0;
}