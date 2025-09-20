from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='demo_nodes_cpp', executable='talker',   name='talker'),
        Node(package='demo_nodes_py',  executable='listener', name='listener'),
        # add your nodes here, e.g. rplidar, motor_ctrl, etc.
    ])
