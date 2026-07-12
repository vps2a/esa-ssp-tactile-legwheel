from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_enable = LaunchConfiguration("auto_enable")
    publish_initial_commands = LaunchConfiguration("publish_initial_commands")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "auto_enable",
                default_value="false",
                description="If true, publish /legwheel/motor_status=true on startup.",
            ),
            DeclareLaunchArgument(
                "publish_initial_commands",
                default_value="true",
                description="Publish initial zero, spring, and damping command arrays on startup.",
            ),
            Node(
                package="legwheel_can",
                executable="legwheel_controller_node",
                name="legwheel_controller",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"auto_enable": auto_enable},
                    {"publish_initial_commands": publish_initial_commands},
                ],
            ),
        ]
    )
