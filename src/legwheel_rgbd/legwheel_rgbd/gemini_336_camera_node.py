#!/usr/bin/env python3

"""Forward Gemini 336 data immediately onto project-stable ROS topics."""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, Imu

from legwheel_rgbd.stream_utils import camera_info_signature


class Gemini336CameraNode(Node):
    """Provide stable topic names while preserving message content and stamps."""

    def __init__(self) -> None:
        super().__init__("gemini_336_camera_node")

        self.declare_parameter("source_rgb_topic", "/legwheel_rgbd/color/image_raw")
        self.declare_parameter(
            "source_rgb_camera_info_topic",
            "/legwheel_rgbd/color/camera_info",
        )
        self.declare_parameter("source_depth_topic", "/legwheel_rgbd/depth/image_raw")
        self.declare_parameter(
            "source_depth_camera_info_topic",
            "/legwheel_rgbd/depth/camera_info",
        )
        self.declare_parameter("source_imu_topic", "/legwheel_rgbd/gyro_accel/sample")

        self.declare_parameter("rgb_topic", "/camera/rgbd/rgb/image_raw")
        self.declare_parameter(
            "rgb_camera_info_topic",
            "/camera/rgbd/rgb/camera_info",
        )
        self.declare_parameter("depth_topic", "/camera/rgbd/depth/image_raw")
        self.declare_parameter(
            "depth_camera_info_topic",
            "/camera/rgbd/depth/camera_info",
        )
        self.declare_parameter("imu_topic", "/camera/imu")

        self._source_topics = {
            "rgb": self.get_parameter("source_rgb_topic").value,
            "rgb_info": self.get_parameter("source_rgb_camera_info_topic").value,
            "depth": self.get_parameter("source_depth_topic").value,
            "depth_info": self.get_parameter("source_depth_camera_info_topic").value,
            "imu": self.get_parameter("source_imu_topic").value,
        }
        self._source_received = {name: False for name in self._source_topics}

        # Image and IMU streams keep sensor-data QoS. CameraInfo is different:
        # it changes rarely, so a transient-local publisher retains the most recent
        # calibration for rosbag and other subscribers that connect later.
        camera_info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.rgb_pub = self.create_publisher(
            Image,
            self.get_parameter("rgb_topic").value,
            qos_profile_sensor_data,
        )
        self.rgb_camera_info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter("rgb_camera_info_topic").value,
            camera_info_qos,
        )
        self.depth_pub = self.create_publisher(
            Image,
            self.get_parameter("depth_topic").value,
            qos_profile_sensor_data,
        )
        self.depth_camera_info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter("depth_camera_info_topic").value,
            camera_info_qos,
        )
        self.imu_pub = self.create_publisher(
            Imu,
            self.get_parameter("imu_topic").value,
            qos_profile_sensor_data,
        )

        # Each high-rate source has its own callback. There is deliberately no
        # RGB/depth synchronizer, queue, timer, or timestamp filter here: waiting
        # for a second stream would add latency and could discard otherwise valid
        # camera data. Publishing the received message object unchanged preserves
        # its original header stamp for synchronization during post-processing.
        self._rgb_subscription = self.create_subscription(
            Image,
            self._source_topics["rgb"],
            self._publish_rgb,
            qos_profile_sensor_data,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            self._source_topics["depth"],
            self._publish_depth,
            qos_profile_sensor_data,
        )
        self._rgb_camera_info_subscription = self.create_subscription(
            CameraInfo,
            self._source_topics["rgb_info"],
            self._publish_rgb_camera_info_if_changed,
            qos_profile_sensor_data,
        )
        self._depth_camera_info_subscription = self.create_subscription(
            CameraInfo,
            self._source_topics["depth_info"],
            self._publish_depth_camera_info_if_changed,
            qos_profile_sensor_data,
        )
        self._imu_subscription = self.create_subscription(
            Imu,
            self._source_topics["imu"],
            self._publish_imu,
            qos_profile_sensor_data,
        )

        self._rgb_camera_info_signature = None
        self._depth_camera_info_signature = None

        self._waiting_log_timer = self.create_timer(5.0, self._log_waiting_sources)

        self.get_logger().info(
            "Gemini 336 bridge forwarding RGB, depth, and IMU independently "
            "as each source message arrives"
        )

    def _publish_rgb(self, message: Image) -> None:
        """Forward one RGB callback immediately, without pairing or filtering."""
        self._source_received["rgb"] = True
        self.rgb_pub.publish(message)

    def _publish_depth(self, message: Image) -> None:
        """Forward one depth callback immediately, without pairing or filtering."""
        self._source_received["depth"] = True
        self.depth_pub.publish(message)

    def _publish_rgb_camera_info_if_changed(self, message: CameraInfo) -> None:
        """Publish RGB calibration initially and whenever its values change."""
        self._source_received["rgb_info"] = True
        signature = camera_info_signature(message)
        if signature == self._rgb_camera_info_signature:
            return
        self._rgb_camera_info_signature = signature
        self.rgb_camera_info_pub.publish(message)

    def _publish_depth_camera_info_if_changed(self, message: CameraInfo) -> None:
        """Publish depth calibration initially and whenever its values change."""
        self._source_received["depth_info"] = True
        signature = camera_info_signature(message)
        if signature == self._depth_camera_info_signature:
            return
        self._depth_camera_info_signature = signature
        self.depth_camera_info_pub.publish(message)

    def _publish_imu(self, message: Imu) -> None:
        """Forward every IMU callback without replaying or filtering messages."""
        self._source_received["imu"] = True
        self.imu_pub.publish(message)

    def _log_waiting_sources(self) -> None:
        """Report vendor topics that have not delivered a message yet."""
        missing_sources = [
            self._source_topics[name]
            for name, received in self._source_received.items()
            if not received
        ]
        if missing_sources:
            source_status = "; ".join(
                self._describe_source_topic(topic) for topic in missing_sources
            )
            self.get_logger().warning(
                "Waiting for Gemini 336 source topics: " + source_status
            )

    def _describe_source_topic(self, topic: str) -> str:
        publishers = self.get_publishers_info_by_topic(topic)
        if not publishers:
            return f"{topic} (no publishers discovered)"

        publisher_names = ", ".join(
            f"{publisher.node_namespace.rstrip('/')}/{publisher.node_name}"
            for publisher in publishers
        )
        return f"{topic} ({len(publishers)} publisher(s): {publisher_names})"


def main(args=None):
    rclpy.init(args=args)
    node = Gemini336CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
