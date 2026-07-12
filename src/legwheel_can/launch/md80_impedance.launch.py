from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motor_ids = PathJoinSubstitution(
        [FindPackageShare("legwheel_can"), "config", "motor_ids.yaml"]
    )
    motor_limits = PathJoinSubstitution(
        [FindPackageShare("legwheel_can"), "config", "motor_limits.yaml"]
    )
    motor_config = PathJoinSubstitution(
        [FindPackageShare("legwheel_can"), "config", "motor_config.json"]
    )

    enable_control = LaunchConfiguration("enable_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_control",
                default_value="false",
                description="Arm impedance control. Motors still require /legwheel/motor_status=true.",
            ),
            Node(
                package="legwheel_can",
                executable="md80_impedance_node",
                name="legwheel_can",
                output="screen",
                parameters=[
                    motor_ids,
                    motor_limits,
                    {"motor_config_json_path": motor_config},
                    {"enable_control": enable_control},
                ],
            ),
        ]
    )
