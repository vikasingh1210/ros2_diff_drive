from setuptools import setup
package_name = 'inertial_distance'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/inertial_distance.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vikas',
    maintainer_email='you@example.com',
    description='IMU (accel+gyro) distance integrator with ZUPT and encoder velocity blend',
    license='MIT',
    entry_points={'console_scripts': [
        'inertial_distance_node = inertial_distance.inertial_distance_node:main',
    ]},
)
