from launch import LaunchDescription #Importing the necessary classes and functions to create a launch description and define nodes
from launch_ros.actions import Node #Importing the Node action to define nodes that will be launched


def generate_launch_description():
    fake_rgbd_node = Node(
        package="legwheel_rgbd",
        executable="fake_rgbd_publisher_node",
        name="fake_rgbd_publisher_node", #Giving the node a name, which can be used to refer to it from other nodes or from the command line
        output="screen",
        parameters=[
            {
                "width": 320,
                "height": 240,
                "rate_hz": 10.0,
            }
        ],
    )

    depth_visualizer_node = Node(
        package="legwheel_rgbd",
        executable="depth_visualizer_node",
        name="depth_visualizer_node",
        output="screen",
        parameters=[
            {
                "min_depth_m": 0.3,
                "max_depth_m": 2.5,
            }
        ],
    )

    return LaunchDescription([
        fake_rgbd_node,
        depth_visualizer_node,
    ])