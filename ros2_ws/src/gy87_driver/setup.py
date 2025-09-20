from setuptools import setup

package_name = 'gy87_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gy87_bringup.launch.py']),
    ],
    install_requires=['setuptools','smbus2','numpy'],
    zip_safe=True,
    maintainer='Vikas',
    maintainer_email='you@example.com',
    description='ROS 2 drivers for GY-87/HW-290 (MPU6050, HMC5883L, BMP180)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mpu6050_node = gy87_driver.mpu6050_node:main',
            'hmc5883l_node = gy87_driver.hmc5883l_node:main',
            'bmp180_node = gy87_driver.bmp180_node:main',
            'qmc5883l_node = gy87_driver.qmc5883l_node:main',   # <--- add this line
        ],
    },
)
