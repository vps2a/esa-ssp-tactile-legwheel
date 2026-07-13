from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motor_config = PathJoinSubstitution(
        [FindPackageShare("legwheel_can"), "config", "motor_config.json"]
    )

    torque_limit = LaunchConfiguration("torque_limit_nm")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "torque_limit_nm",
                default_value="8.0",
                description="Controller-side wheel torque clamp in Nm.",
            ),
            Node(
                package="legwheel_can",
                executable="wheel_controller_node",
                name="wheel_controller",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"motor_config_json_path": motor_config},
                    {"torque_limit_nm": torque_limit},
                ],
            ),
        ]
    )
