import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("ros2_diff_drive_bridge"),
        "config",
        "params.yaml",
    )
    return LaunchDescription([
        Node(
            package="ros2_diff_drive_bridge",
            executable="serial_bridge",
            name="arduino_diffdrive_bridge",
            output="screen",
            parameters=[cfg],
        )
    ])
