#!/usr/bin/env python3

import struct

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

        self.sub = self.create_subscription(
            Image,
            input_depth_topic,
            self.depth_callback,
            10,
        )

        self.pub = self.create_publisher(
            Image,
            output_depth_viz_topic,
            10,
        )

        self.get_logger().info(
            f"Depth visualizer: {input_depth_topic} -> {output_depth_viz_topic}"
        )

        self.get_logger().info(
            f"Depth range: {self.min_depth_m:.2f} m to {self.max_depth_m:.2f} m"
        )

    def depth_callback(self, msg):
        if msg.encoding not in ["32FC1", "16UC1"]:
            if not self.has_warned_about_encoding:
                self.get_logger().warning(
                    f"Unsupported depth encoding: {msg.encoding}. "
                    "Expected 32FC1 or 16UC1."
                )
                self.has_warned_about_encoding = True
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

        if depth_range <= 0.0:
            self.get_logger().error("max_depth_m must be greater than min_depth_m")
            return

        if msg.encoding == "32FC1":
            output = self.convert_32fc1_to_mono8(msg, depth_range)

        elif msg.encoding == "16UC1":
            output = self.convert_16uc1_to_mono8(msg, depth_range)

        viz.data = bytes(output)
        self.pub.publish(viz)

    def convert_32fc1_to_mono8(self, msg, depth_range):
        output = bytearray()

        expected_bytes = msg.width * msg.height * 4
        if len(msg.data) < expected_bytes:
            self.get_logger().warning(
                f"Depth image data too short for 32FC1: "
                f"got {len(msg.data)} bytes, expected {expected_bytes}"
            )
            return output

        for i in range(0, expected_bytes, 4):
            depth_m = struct.unpack_from("f", msg.data, i)[0]
            output.append(self.depth_m_to_pixel(depth_m, invalid_if_zero=True))

        return output

    def convert_16uc1_to_mono8(self, msg, depth_range):
        output = bytearray()

        expected_bytes = msg.width * msg.height * 2
        if len(msg.data) < expected_bytes:
            self.get_logger().warning(
                f"Depth image data too short for 16UC1: "
                f"got {len(msg.data)} bytes, expected {expected_bytes}"
            )
            return output

        for i in range(0, expected_bytes, 2):
            raw_depth_mm = struct.unpack_from("H", msg.data, i)[0]

            # Convention: 16UC1 depth is usually in millimetres.
            # 1000 means approximately 1.0 metre.
            depth_m = raw_depth_mm / 1000.0

            output.append(self.depth_m_to_pixel(depth_m, invalid_if_zero=True))

        return output

    def depth_m_to_pixel(self, depth_m, invalid_if_zero=True):
        if invalid_if_zero and depth_m <= 0.0:
            return 0

        normalized = (depth_m - self.min_depth_m) / (
            self.max_depth_m - self.min_depth_m
        )

        normalized = max(0.0, min(1.0, normalized))

        # Near objects are bright, far objects are dark.
        pixel = int(255 * (1.0 - normalized))

        return pixel


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