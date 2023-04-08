#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <thread>


class SimpleSerialReceiver : public rclcpp::Node
{
public:
  SimpleSerialReceiver() : Node("simple_serial_receiver")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 115200);

    port_ = get_parameter("port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();

    pub_ = create_publisher<std_msgs::msg::String>("simple_receiver", 10);
  }

  void execute()
  {
    while(rclcpp::ok())
    {
        auto message = std_msgs::msg::String();
        message.data = "data";
        pub_->publish(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::string port_;
  int baudrate_;
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
