from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="motors",
                executable="motor_control",
                parameters=[{"dev": True}],
            ),
            Node(package="motors", executable="motor_current"),
            Node(package="motors", executable="relay_control"),
            Node(
                package="motors",
                executable="hand_controller",
                parameters=[{"dev": True}],
            ),
        ]
    )
