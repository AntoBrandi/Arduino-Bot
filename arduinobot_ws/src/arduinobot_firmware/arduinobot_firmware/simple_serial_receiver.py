#!/usr/bin/env python3
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.pub_ = self.create_publisher(String, "simple_receiver", 10)
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

    def execute(self):
        while rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline()

            try:
                data.decode("utf-8")
            except:
                continue
            
            self.get_logger().info("New data available on serial, publishing on ROS 2 topic")
            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)
            time.sleep(0.01)


def main():
    rclpy.init()

    simple_serial_transmitter = SimpleSerialReceiver()
    simple_serial_transmitter.execute()
    rclpy.spin(simple_serial_transmitter)
    
    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()