"""
Launch file for Move Robot GUI.

This launches the move_robot_gui node which provides a GUI for controlling
the virtual robot base movement.

Usage:
    ros2 launch move_robot move_robot_gui.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='move_robot',
            executable='move_robot_gui',
            name='move_robot_gui',
            output='screen',
        ),
    ])
