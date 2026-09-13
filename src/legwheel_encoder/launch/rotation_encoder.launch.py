from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare("legwheel_encoder"),
        "config",
        "rotation_encoder.yaml",
    ])

    return LaunchDescription([
        Node(
            package="legwheel_encoder",
            executable="rotation_encoder_node",
            name="rotation_encoder_node",
            parameters=[config_path],
            output="screen",
        )
    ])