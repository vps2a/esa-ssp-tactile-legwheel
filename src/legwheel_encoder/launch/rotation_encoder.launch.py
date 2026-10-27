"""Read every encoder setting from the selected experiment configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "experiment_config",
                description="Path to the experiment YAML used by the experiment runner.",
            ),
            Node(
                package="legwheel_encoder",
                executable="rotation_encoder_node",
                name="rotation_encoder_node",
                parameters=[
                    {"experiment_config": LaunchConfiguration("experiment_config")}
                ],
                output="screen",
            ),
        ]
    )
