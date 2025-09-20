from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyACM0",
        description="Serial port of Arduino"
    )
    baud_arg = DeclareLaunchArgument(
        "baud", default_value="115200",
        description="Baud rate"
    )
    radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.03")
    base_arg   = DeclareLaunchArgument("wheel_base",   default_value="0.16")
    tprl_arg   = DeclareLaunchArgument("ticks_per_rev_left",  default_value="620")
    tprr_arg   = DeclareLaunchArgument("ticks_per_rev_right", default_value="620")
    idle_arg   = DeclareLaunchArgument("idle_timeout_ms", default_value="500")

    node = Node(
        package="arduino_bridge",
        executable="serial_bridge",
        name="arduino_bridge",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baud": LaunchConfiguration("baud"),
            "wheel_radius": LaunchConfiguration("wheel_radius"),
            "wheel_base": LaunchConfiguration("wheel_base"),
            "ticks_per_rev_left": LaunchConfiguration("ticks_per_rev_left"),
            "ticks_per_rev_right": LaunchConfiguration("ticks_per_rev_right"),
            "idle_timeout_ms": LaunchConfiguration("idle_timeout_ms"),
            "read_hz": 100.0,
            "rpm_hz": 20.0,
            "publish_arduino_raw": False,
        }],
    )

    return LaunchDescription([port_arg, baud_arg, radius_arg, base_arg, tprl_arg, tprr_arg, idle_arg, node])
