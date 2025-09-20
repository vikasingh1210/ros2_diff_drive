from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gyro_distance',
            executable='gyro_distance_node',
            name='gyro_distance',
            output='screen',
            parameters=[{
                # Leave blank to auto-discover any sensor_msgs/Imu topic
                'imu_topic': '',

                # Mounting (choose robot-forward axis on the IMU)
                'forward_axis': 'x',    # 'x' or 'y'
                'axis_sign': 1,         # +1 or -1

                # Filters / stability
                'accel_lpf_hz': 8.0,
                'accel_deadband': 0.05,   # tighter noise kill

                # ZUPT settings
                'use_zupt': True,
                'zupt_acc_eps': 0.03,     # m/s^2
                'zupt_gyro_eps': 0.2,     # rad/s (more forgiving)
                'zupt_hold_ms': 200,
                'bias_learn_alpha': 0.02,

                # Scaling
                'accel_scale': 1.0,       # auto-cal will update
                'auto_accel_scale': True,
                'auto_calib_duration_s': 2.0,

                # Gyro scaling (assume driver outputs deg/s → convert to rad/s)
                # If your driver already outputs rad/s, set this to 1.0
                'gyro_scale': 0.0174533,

                # Initial attitude (roll/pitch) from gravity at startup
                'assume_still_on_start': True,
                'init_att_calib_s': 1.5,

                # Movement gating (don’t integrate unless moving)
                'integrate_only_when_moving': True,
                'start_accel_thresh': 0.10,  # m/s^2 to start integrating
            }]
        )
    ])
