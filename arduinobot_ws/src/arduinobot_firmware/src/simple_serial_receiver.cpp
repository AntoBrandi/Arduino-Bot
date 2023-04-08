#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <thread>

#include <libserial/SerialPort.h>


class SimpleSerialReceiver : public rclcpp::Node
{
public:
  SimpleSerialReceiver() : Node("simple_serial_receiver")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");

    port_ = get_parameter("port").as_string();

    pub_ = create_publisher<std_msgs::msg::String>("serial_receiver", 10);

    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }

  ~SimpleSerialReceiver()
  {
    arduino_.Close();
  }

  void execute()
  {
    while(rclcpp::ok())
    {
      if(arduino_.IsDataAvailable())
      {
        auto message = std_msgs::msg::String();
        arduino_.ReadLine(message.data);
        pub_->publish(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::string port_;
  int baudrate_;
  LibSerial::SerialPort arduino_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSerialReceiver>();
  node->execute();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
