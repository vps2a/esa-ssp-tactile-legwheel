"""Launch the LegWheel track overlay for live data or bag playback."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    experiment_config = LaunchConfiguration("experiment_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "experiment_config",
                description="Absolute path to the experiment YAML configuration.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Set true while replaying a rosbag with --clock.",
            ),
            DeclareLaunchArgument(
                "publish_kinematic_state",
                default_value="true",
                description="Start the shared IMU pose publisher; false when already running.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/legwheel_rgbd/gyro_accel/sample",
                description="Recorded camera IMU topic; use /camera/imu for legacy bags.",
            ),
            DeclareLaunchArgument(
                "wheel_topic",
                default_value="/wheel/wheel_state",
                description="Measured wheel velocity for encoder yaw refinement.",
            ),
            DeclareLaunchArgument(
                "encoder_topic",
                default_value="/rotation_encoder/ticks",
                description="Central encoder Int64 ticks topic.",
            ),
            DeclareLaunchArgument(
                "encoder_bag",
                default_value="",
                description="Playback bag directory; defaults to rosbag beside the experiment snapshot.",
            ),
            Node(
                package="legwheel_kinematics",
                executable="kinematic_joint_state_node",
                name="kinematic_joint_state",
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_kinematic_state")),
                parameters=[
                    {
                        "experiment_config": experiment_config,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "encoder_topic": LaunchConfiguration("encoder_topic"),
                        "encoder_bag": LaunchConfiguration("encoder_bag"),
                        "wheel_topic": LaunchConfiguration("wheel_topic"),
                    }
                ],
            ),
            Node(
                package="legwheel_kinematics",
                executable="track_overlay_node",
                name="track_overlay",
                output="screen",
                parameters=[
                    {
                        "experiment_config": experiment_config,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
        ]
    )
