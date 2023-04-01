import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):

    def __init__(self):
        super().__init__("simple_publisher")
        self.pub_ = self.create_publisher(String, "chatter", 10)
        self.counter_ = 0
        self.get_logger().info("Node Publisher Ready")

    def execute(self, rate):
        self.get_logger().info("Publishing at %dHz" % rate)

        rate = self.create_rate(rate)
        while rclpy.ok():
            msg = String()
            msg.data = "Hello ROS 2 - counter: %d" % self.counter_
            self.pub_.publish(msg)
            self.counter_ += 1
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()
    simple_publisher.execute(1)

    rclpy.spin(simple_publisher)

    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
