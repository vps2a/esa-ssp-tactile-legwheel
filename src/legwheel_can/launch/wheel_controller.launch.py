from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motor_config = PathJoinSubstitution(
        [FindPackageShare("legwheel_can"), "config", "motor_config.json"]
    )

    speed_limit = LaunchConfiguration("speed_limit_rad_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "speed_limit_rad_s",
                default_value="2.0",
                description="Controller-side wheel speed clamp in rad/s.",
            ),
            Node(
                package="legwheel_can",
                executable="wheel_controller_node",
                name="wheel_controller",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"motor_config_json_path": motor_config},
                    {"speed_limit_rad_s": speed_limit},
                ],
            ),
        ]
    )
