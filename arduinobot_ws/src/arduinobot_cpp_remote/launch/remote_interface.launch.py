from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    task_server_node = Node(
        package="arduinobot_cpp_remote",
        executable="task_server",
    )

    alexa_interface_node = Node(
        package="arduinobot_py_remote",
        executable="alexa_interface",
    )

    return LaunchDescription([
        task_server_node,
        alexa_interface_node
    ])