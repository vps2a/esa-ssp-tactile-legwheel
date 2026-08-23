from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_path = LaunchConfiguration("config_file_path")
    camera_name = LaunchConfiguration("camera_name")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    serial_number = LaunchConfiguration("serial_number")
    usb_port = LaunchConfiguration("usb_port")
    camera_config = LaunchConfiguration("camera_config")

    default_config = PathJoinSubstitution([
        FindPackageShare("legwheel_rgbd"),
        "config",
        "gemini_336.yaml",
    ])

    orbbec_launch = PathJoinSubstitution([
        FindPackageShare("orbbec_camera"),
        "launch",
        "gemini_330_series.launch.py",
    ])

    default_camera_config = PathJoinSubstitution([
        FindPackageShare("legwheel_rgbd"),
        "config",
        "camera_config.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file_path",
            default_value=default_config,
            description="YAML parameter file passed to the Orbbec Gemini 330-series launch file.",
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="legwheel_rgbd",
            description="ROS namespace used by the Orbbec camera node.",
        ),
        DeclareLaunchArgument(
            "enable_point_cloud",
            default_value="false",
            description="Set true to publish the Orbbec point cloud stream.",
        ),
        DeclareLaunchArgument(
            "serial_number",
            default_value="",
            description="Optional camera serial number for multi-camera systems.",
        ),
        DeclareLaunchArgument(
            "usb_port",
            default_value="",
            description="Optional USB port selector for multi-camera systems.",
        ),
        DeclareLaunchArgument(
            "camera_config",
            default_value=default_camera_config,
            description="LegWheel camera bridge config containing publish rates and topics.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch),
            launch_arguments={
                "config_file_path": config_file_path,
                "camera_name": camera_name,
                "enable_point_cloud": enable_point_cloud,
                "serial_number": serial_number,
                "usb_port": usb_port,
            }.items(),
        ),
        Node(
            package="legwheel_rgbd",
            executable="gemini_336_camera_node",
            name="gemini_336_camera_node",
            parameters=[camera_config],
            output="screen",
        ),
    ])
