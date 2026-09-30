"""Launch the LegWheel track overlay for live data or bag playback."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    experiment_config = LaunchConfiguration("experiment_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "experiment_config",
            description="Absolute path to the experiment YAML configuration.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true while replaying a rosbag with --clock.",
        ),
        Node(
            package="legwheel_kinematics",
            executable="track_overlay_node",
            name="track_overlay",
            output="screen",
            parameters=[{
                "experiment_config": experiment_config,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
