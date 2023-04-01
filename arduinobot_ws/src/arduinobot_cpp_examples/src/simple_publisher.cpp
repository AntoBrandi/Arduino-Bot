#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher() : Node("simple_publisher"), counter_(0)
  {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    RCLCPP_INFO(get_logger(), "Node Publisher Ready");
  }

  void execute(int rate)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Publishing at " << rate << "Hz");
    rclcpp::Rate loop_rate(rate);
    while (rclcpp::ok())
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello ROS 2 - counter:" + std::to_string(counter_++);
      pub_->publish(message);
      loop_rate.sleep();
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  unsigned int counter_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimplePublisher>();
  node->execute(1);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}