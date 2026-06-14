#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthVisualizerNode(Node):
    def __init__(self):
        super().__init__("depth_visualizer_node")

        self.declare_parameter("input_depth_topic", "/iphone/depth/image_raw")
        self.declare_parameter("output_depth_viz_topic", "/iphone/depth/image_viz")
        self.declare_parameter("min_depth_m", 0.3)
        self.declare_parameter("max_depth_m", 2.5)

        input_depth_topic = self.get_parameter("input_depth_topic").value
        output_depth_viz_topic = self.get_parameter("output_depth_viz_topic").value

        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.has_warned_about_encoding = False

        # Queue size 1 is important for low latency.
        # If the node is briefly slower than the incoming stream,
        # old frames are dropped instead of displayed late.
        self.sub = self.create_subscription(
            Image,
            input_depth_topic,
            self.depth_callback,
            1,
        )

        self.pub = self.create_publisher(
            Image,
            output_depth_viz_topic,
            1,
        )

        self.get_logger().info(
            f"Depth visualizer: {input_depth_topic} -> {output_depth_viz_topic}"
        )
        self.get_logger().info(
            f"Depth range: {self.min_depth_m:.2f} m to {self.max_depth_m:.2f} m"
        )

    def depth_callback(self, msg):
        if self.max_depth_m <= self.min_depth_m:
            self.get_logger().error("max_depth_m must be greater than min_depth_m")
            return

        if msg.encoding == "16UC1":
            depth_raw = np.frombuffer(msg.data, dtype=np.uint16)
            depth_m = depth_raw.astype(np.float32) / 1000.0

        elif msg.encoding == "32FC1":
            depth_m = np.frombuffer(msg.data, dtype=np.float32)

        else:
            if not self.has_warned_about_encoding:
                self.get_logger().warning(
                    f"Unsupported depth encoding: {msg.encoding}. "
                    "Expected 16UC1 or 32FC1."
                )
                self.has_warned_about_encoding = True
            return

        expected_pixels = msg.width * msg.height
        if depth_m.size < expected_pixels:
            self.get_logger().warning(
                f"Depth image has too few pixels: got {depth_m.size}, "
                f"expected {expected_pixels}"
            )
            return

        depth_m = depth_m[:expected_pixels].reshape((msg.height, msg.width))

        normalized = (depth_m - self.min_depth_m) / (
            self.max_depth_m - self.min_depth_m
        )
        normalized = np.clip(normalized, 0.0, 1.0)

        # Invalid zero-depth pixels become black.
        valid = depth_m > 0.0

        # Near = bright, far = dark.
        mono = (255.0 * (1.0 - normalized)).astype(np.uint8)
        mono[~valid] = 0

        viz = Image()
        viz.header = msg.header
        viz.height = msg.height
        viz.width = msg.width
        viz.encoding = "mono8"
        viz.is_bigendian = False
        viz.step = msg.width
        viz.data = mono.tobytes()

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