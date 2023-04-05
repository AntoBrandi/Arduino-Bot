import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("arduinobot", package_name="arduinobot_moveit")
        .robot_description(
            file_path="config/arduinobot.urdf.xacro" # TODO: take from arduinobot_description
        )
        .robot_description_semantic(file_path="config/arduinobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml") # TODO: take from arduinobot_controller
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("arduinobot_moveit"),
            "config",
            "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            move_group_node, 
            rviz_node
        ]
    )