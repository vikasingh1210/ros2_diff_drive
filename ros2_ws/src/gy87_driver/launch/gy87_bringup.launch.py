from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gy87_driver',
            executable='mpu6050_node',
            name='mpu6050',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'address': 0x68,
                'frame_id': 'imu_link',
                'rate_hz': 100.0,
                'accel_bias': [0.0,0.0,0.0],
                'accel_scale':[1.0,1.0,1.0],
                'gyro_bias' : [0.0,0.0,0.0],
                'gyro_scale': [1.0,1.0,1.0],
                'remove_gravity': True
            }]
        ),
        # Swap HMC → QMC:
        Node(
            package='gy87_driver',
            executable='qmc5883l_node',     # <--- changed
            name='qmc5883l',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'address': 0x0D,            # <--- QMC address
                'frame_id': 'imu_link',
                'rate_hz': 50.0,
            }]
        ),
        Node(
            package='gy87_driver',
            executable='bmp180_node',
            name='bmp180',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'address': 0x77,
                'frame_id': 'imu_link',
                'rate_hz': 10.0
            }]
        ),
    ])
