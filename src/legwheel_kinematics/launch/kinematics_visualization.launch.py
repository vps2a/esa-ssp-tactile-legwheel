"""Launch replay-time TF and 3D point visualization for Foxglove."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    experiment_config = LaunchConfiguration("experiment_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    track_point_count = LaunchConfiguration("track_point_count")

    return LaunchDescription([
        DeclareLaunchArgument(
            "experiment_config",
            description="Absolute path to the recorded experiment YAML snapshot.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the clock published by rosbag playback.",
        ),
        DeclareLaunchArgument(
            "track_point_count",
            default_value="360",
            description="Number of points in each circular track boundary.",
        ),
        Node(
            package="legwheel_kinematics",
            executable="kinematics_visualization_node",
            name="kinematics_visualization",
            output="screen",
            parameters=[{
                "experiment_config": experiment_config,
                "track_point_count": ParameterValue(
                    track_point_count,
                    value_type=int,
                ),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
