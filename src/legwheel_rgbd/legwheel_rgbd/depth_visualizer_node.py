#!/usr/bin/env python3

import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthVisualizerNode(Node):
    def __init__(self):
        super().__init__("depth_visualizer_node")

        self.declare_parameter("min_depth_m", 0.3)
        self.declare_parameter("max_depth_m", 2.5)

        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.sub = self.create_subscription(
            Image,
            "/iphone/depth/image_raw",
            self.depth_callback,
            10,
        )

        self.pub = self.create_publisher(
            Image,
            "/iphone/depth/image_viz",
            10,
        )

        self.get_logger().info("Depth visualizer started")

    def depth_callback(self, msg):
        if msg.encoding != "32FC1":
            self.get_logger().warn_once(
                f"Expected 32FC1 depth image, got {msg.encoding}"
            )
            return

        viz = Image()
        viz.header = msg.header
        viz.height = msg.height
        viz.width = msg.width
        viz.encoding = "mono8"
        viz.is_bigendian = False
        viz.step = msg.width

        output = bytearray()
        depth_range = self.max_depth_m - self.min_depth_m

        for i in range(0, len(msg.data), 4):
            depth_m = struct.unpack("f", msg.data[i:i + 4])[0]

            normalized = (depth_m - self.min_depth_m) / depth_range
            normalized = max(0.0, min(1.0, normalized))

            pixel = int(255 * (1.0 - normalized))
            output.append(pixel)

        viz.data = bytes(output)
        self.pub.publish(viz)


def main(args=None):
    rclpy.init(args=args)
    node = DepthVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()