from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inertial_distance',
            executable='inertial_distance_node',
            name='inertial_distance',
            output='screen',
            parameters=[{
                'imu_topic': '/imu',
                'rpm_topic': '/wheel_rpm',
                'use_rpm_for_zupt': True,
                'zupt_vel_epsilon': 0.02,
                'zupt_hold_ms': 150,
                'beta_enc_blend': 0.05,
                'rate_hz': 100.0,
                'wheel_radius': 0.03,
                'distance_topic': '/inertial_distance',
                'odom_topic': '/odom_inertial',
                'frame_id': 'odom',
                'base_frame_id': 'base_link',
            }],
        )
    ])
