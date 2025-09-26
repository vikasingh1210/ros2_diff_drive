from setuptools import setup
package_name = "ros2_diff_drive_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bridge.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vikas Singh",
    maintainer_email="vikasingh1210@gmail.com",
    description="ROS 2 ↔ Arduino serial bridge for diff-drive.",
    license="MIT",
    entry_points={"console_scripts": [
        "serial_bridge = ros2_diff_drive_bridge.serial_bridge:main",
    ]},
)
