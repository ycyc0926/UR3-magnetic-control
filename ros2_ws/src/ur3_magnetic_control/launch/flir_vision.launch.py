import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("ur3_magnetic_control"),
        "config",
        "flir_vision.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="ur3_magnetic_control",
                executable="camera_node",
                name="camera_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="ur3_magnetic_control",
                executable="target_tracker",
                name="target_tracker",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="ur3_magnetic_control",
                executable="safety_supervisor",
                name="safety_supervisor",
                parameters=[config],
                output="screen",
            ),
        ]
    )
