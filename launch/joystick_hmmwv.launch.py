from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("chrono_sim_control"),
            "config",
            "chrono_joy_driver.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="YAML configuration for the Chrono joystick driver.",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
            Node(
                package="chrono_sim_control",
                executable="chrono_joy_driver",
                name="chrono_joy_driver",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
