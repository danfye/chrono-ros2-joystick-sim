# Copyright 2026 Chrono ROS 2 Demo Maintainer
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    headless = LaunchConfiguration("headless")
    max_runtime_seconds = LaunchConfiguration("max_runtime_seconds")
    chrono_data_path = LaunchConfiguration("chrono_data_path")
    start_joy_node = LaunchConfiguration("start_joy_node")

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
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run the Chrono driver without opening an Irrlicht window.",
            ),
            DeclareLaunchArgument(
                "max_runtime_seconds",
                default_value="0.0",
                description=(
                    "Stop the Chrono driver after this simulation time; "
                    "0 means unlimited."
                ),
            ),
            DeclareLaunchArgument(
                "start_joy_node",
                default_value="true",
                description="Start joy_node alongside the Chrono driver.",
            ),
            DeclareLaunchArgument(
                "chrono_data_path",
                default_value="/usr/local/share/chrono/data/",
                description="Project Chrono data directory.",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                condition=IfCondition(start_joy_node),
            ),
            Node(
                package="chrono_sim_control",
                executable="chrono_joy_driver",
                name="chrono_joy_driver",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "headless": ParameterValue(headless, value_type=bool),
                        "max_runtime_seconds": ParameterValue(
                            max_runtime_seconds,
                            value_type=float,
                        ),
                        "chrono_data_path": chrono_data_path,
                    },
                ],
            ),
        ]
    )
