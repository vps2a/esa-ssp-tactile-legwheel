"""Publish the frame tree from the same experiment geometry as processing."""

import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

from ament_index_python.packages import get_package_share_directory
from legwheel_kinematics.config import KinematicsConfig


def _launch_nodes(context):
    config_path = Path(LaunchConfiguration("experiment_config").perform(context))
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower()
    use_sim_time = use_sim_time in ("true", "1", "yes")
    config = KinematicsConfig.from_yaml(config_path)
    xacro_path = (
        Path(get_package_share_directory("legwheel_description"))
        / "urdf"
        / "legwheel.urdf.xacro"
    )
    robot_description = xacro.process_file(
        str(xacro_path),
        mappings={
            "d1": str(config.d1),
            "a_b": str(config.a_b),
            "sensor_front_offset": str(config.sensor_front_offset),
            "camera_height": str(config.camera_height),
            "camera_angle_rad": str(math.radians(config.camera_angle_deg)),
            "h_b": str(config.h_b),
            "camera_beam_offset": str(config.camera_beam_offset),
        },
    ).toxml()
    return [
        Node(
            package="legwheel_kinematics",
            executable="kinematic_joint_state_node",
            parameters=[
                {
                    "experiment_config": str(config_path),
                    "use_sim_time": use_sim_time,
                    "imu_topic": LaunchConfiguration("imu_topic").perform(context),
                    "encoder_topic": LaunchConfiguration("encoder_topic").perform(
                        context
                    ),
                }
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": use_sim_time,
                    # The shared source already orders and validates each pose.
                    # Accept its delayed acquisition stamps, including after a
                    # playback rewind, instead of throttling on an old timestamp.
                    "ignore_timestamp": True,
                }
            ],
            remappings=[
                ("/joint_states", "/legwheel/kinematic_joint_states"),
            ],
        ),
    ]


def generate_launch_description():
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
                "imu_topic",
                default_value="/legwheel_rgbd/gyro_accel/sample",
                description="Combined camera IMU; use /camera/imu for legacy recordings.",
            ),
            DeclareLaunchArgument(
                "encoder_topic",
                default_value="/rotation_encoder/joint_state",
                description="Central encoder JointState used for yaw and acceleration correction.",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
