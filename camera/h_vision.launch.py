"""Start camera + H observations only. No UR or motor command publishers."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    config = '/home/yc/UR3/ros2_ws/src/ur3_magnetic_control/config/flir_vision.yaml'
    camera = ExecuteProcess(cmd=['/usr/bin/python3', '-m', 'ur3_magnetic_control.camera_node',
                                 '--ros-args', '--params-file', config], output='screen')
    tracker = ExecuteProcess(cmd=['/usr/bin/python3', '-m', 'ur3_magnetic_control.h_tracker_node'], output='screen')
    return LaunchDescription([
        camera, tracker,
        RegisterEventHandler(OnProcessExit(target_action=tracker,
            on_exit=[EmitEvent(event=Shutdown(reason='H viewer closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=camera,
            on_exit=[EmitEvent(event=Shutdown(reason='Camera stream ended'))])),
    ])
