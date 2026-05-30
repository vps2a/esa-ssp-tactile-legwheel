from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    depth_visualizer_node = Node(
        package="legwheel_rgbd",
        executable="depth_visualizer_node",
        name="depth_visualizer_node",
        output="screen",
        parameters=[
            {
                "input_depth_topic": "/depth_image",
                "output_depth_viz_topic": "/iphone/depth/image_viz",
                "min_depth_m": 0.2,
                "max_depth_m": 3.0,
            }
        ],
    )

    return LaunchDescription([
        depth_visualizer_node,
    ])