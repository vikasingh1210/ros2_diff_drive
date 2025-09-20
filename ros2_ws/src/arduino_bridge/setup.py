from setuptools import setup
package_name = 'arduino_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arduino_bridge.launch.py']),
    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='Vikas',
    maintainer_email='you@example.com',
    description='ROS2 <-> Arduino serial bridge for differential drive',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = arduino_bridge.serial_bridge:main',
        ],
    },
)
