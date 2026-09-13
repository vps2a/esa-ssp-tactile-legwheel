#!/usr/bin/env python3

import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthVisualizerNode(Node):
    def __init__(self):
        super().__init__("depth_visualizer_node")

        self.declare_parameter("min_depth_m", 0.3)
        self.declare_parameter("max_depth_m", 2.5)
        self.declare_parameter("depth_topic", "/camera/rgbd/depth/image_raw")
        self.declare_parameter("visualization_topic", "/camera/rgbd/depth/image_viz")

        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        depth_topic = self.get_parameter("depth_topic").value
        visualization_topic = self.get_parameter("visualization_topic").value

        self.sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(
            Image,
            visualization_topic,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Depth visualizer started on {depth_topic}")

    def depth_callback(self, msg):
        if msg.encoding not in ("32FC1", "16UC1"):
            self.get_logger().warn_once(
                f"Expected 32FC1 or 16UC1 depth image, got {msg.encoding}"
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

        if msg.encoding == "32FC1":
            depth_values = (
                struct.unpack("f", msg.data[i:i + 4])[0]
                for i in range(0, len(msg.data), 4)
            )
        else:
            depth_values = (
                struct.unpack("H", msg.data[i:i + 2])[0] * 0.001
                for i in range(0, len(msg.data), 2)
            )

        for depth_m in depth_values:
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
