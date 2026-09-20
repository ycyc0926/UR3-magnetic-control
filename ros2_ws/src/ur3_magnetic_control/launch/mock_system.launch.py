import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("ur3_magnetic_control"),
        "config",
        "mock_system.yaml",
    )
    nodes = []
    for executable in (
        "camera_node",
        "target_tracker",
        "path_publisher",
        "mock_motor",
        "safety_supervisor",
    ):
        nodes.append(
            Node(
                package="ur3_magnetic_control",
                executable=executable,
                name=executable,
                parameters=[config],
                output="screen",
            )
        )
    return LaunchDescription(nodes)
