#include "arduinobot_controller/arduinobot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace arduinobot_controller
{
ArduinobotInterface::ArduinobotInterface()
{
}


ArduinobotInterface::~ArduinobotInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn ArduinobotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ArduinobotInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  position_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> ArduinobotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> ArduinobotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn ArduinobotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Starting robot hardware ...");

  // Reset commands and states
  position_commands_ = { 0.0, 0.0, 0.0, 0.0 };
  position_states_ = { 0.0, 0.0, 0.0, 0.0 };

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn ArduinobotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type ArduinobotInterface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
  // Open Loop Control - assuming the robot is always where we command to be
  position_states_ = position_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinobotInterface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{
  std::string msg;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (i > 0)
    {
      msg.append(",");  // Delimiter
    }
    msg.append(std::to_string(position_commands_.at(i)));
  }

  try
  {
    arduino_.Write(msg);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                        "Something went wrong while sending the message "
                        << msg << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace arduinobot_controller

PLUGINLIB_EXPORT_CLASS(
  arduinobot_controller::ArduinobotInterface,
  hardware_interface::SystemInterface
)