"""Start the calibrated UR3 driver and MoveIt together."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    system = yaml.safe_load((root / "config/ur3_system.yaml").read_text())
    calibration = system["robot"]["calibration_file"]
    driver = Path(get_package_share_directory("ur_robot_driver")) / "launch/ur_control.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("start_moveit", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver)),
            condition=IfCondition(LaunchConfiguration("start_driver")),
            launch_arguments={
                "ur_type": "ur3",
                "robot_ip": system["network"]["robot_ip"],
                "kinematics_params_file": calibration,
                "launch_rviz": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(root / "robot/calibrated_moveit.launch.py")),
            condition=IfCondition(LaunchConfiguration("start_moveit")),
            launch_arguments={
                "ur_type": "ur3",
                "kinematics_params_file": calibration,
                "launch_rviz": "false",
            }.items(),
        ),
    ])
