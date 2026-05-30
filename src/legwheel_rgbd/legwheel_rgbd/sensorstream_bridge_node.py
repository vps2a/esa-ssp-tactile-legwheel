#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class SensorStreamBridgeNode(Node):
    def __init__(self):
        super().__init__("sensorstream_bridge_node")

        self.declare_parameter("connection", "usb")
        self.declare_parameter("rgb_topic", "/iphone/rgb/image_raw")
        self.declare_parameter("depth_topic", "/iphone/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/iphone/rgb/camera_info")
        self.declare_parameter("imu_topic", "/iphone/imu")

        connection = self.get_parameter("connection").value

        self.get_logger().info(
            f"SensorStream bridge placeholder started with connection={connection}"
        )

        self.get_logger().warn(
            "This node is a scaffold. Next step: connect it to actual SensorStream USB output."
        )


def main(args=None):
    rclpy.init(args=args)
    node = SensorStreamBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()