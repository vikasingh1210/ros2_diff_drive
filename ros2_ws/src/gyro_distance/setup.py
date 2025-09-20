from setuptools import setup

package_name = 'gyro_distance'

setup(
    name=package_name,
    version='0.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gyro_distance.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vikas',
    maintainer_email='you@example.com',
    description='Distance estimation from GY-87 IMU with gravity compensation, ZUPT, auto IMU topic discovery, and startup attitude/scale calibration.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gyro_distance_node = gyro_distance.gyro_distance_node:main',
        ],
    },
)
