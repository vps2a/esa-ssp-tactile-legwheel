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

    enable_control = LaunchConfiguration("enable_control")
    controller_auto_enable = LaunchConfiguration("controller_auto_enable")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_control",
                default_value="false",
                description="Arm impedance control. Motors still require /legwheel/motor_status=true.",
            ),
            DeclareLaunchArgument(
                "controller_auto_enable",
                default_value="false",
                description="If true, controller publishes /legwheel/motor_status=true on startup.",
            ),
            Node(
                package="legwheel_can",
                executable="md80_impedance_node",
                name="legwheel_can",
                output="screen",
                parameters=[
                    motor_ids,
                    motor_limits,
                    {"enable_control": enable_control},
                ],
            ),
            Node(
                package="legwheel_can",
                executable="legwheel_controller_node",
                name="legwheel_controller",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"auto_enable": controller_auto_enable},
                    {"publish_initial_commands": True},
                ],
            ),
        ]
    )
