from setuptools import setup
package_name = 'my_bringup'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
               ('share/' + package_name + '/launch', ['launch/multi.launch.py'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ed',
    description='Bringup for Pi5',
    entry_points={},
)
