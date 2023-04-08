#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


using std::placeholders::_1;

class SimpleSerialTransmitter : public rclcpp::Node
{
public:
  SimpleSerialTransmitter() : Node("simple_serial_transmitter")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 115200);

    port_ = get_parameter("port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();

    sub_ = create_subscription<std_msgs::msg::String>(
        "simple_transmitter", 10, std::bind(&SimpleSerialTransmitter::msgCallback, this, _1));
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string port_;
  int baudrate_;

  void msgCallback(const std_msgs::msg::String &msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "New message received, publishing on serial: ");
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSerialTransmitter>());
  rclcpp::shutdown();
  return 0;
}
