from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    motor_ids = PathJoinSubstitution(
        [FindPackageShare("legwheel_can"), "config", "motor_ids.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="legwheel_can",
                executable="md80_zero_node",
                name="legwheel_can",
                output="screen",
                parameters=[motor_ids],
            )
        ]
    )
