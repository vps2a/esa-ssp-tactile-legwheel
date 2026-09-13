#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu


class Gemini336CameraNode(Node):
    def __init__(self):
        super().__init__("gemini_336_camera_node")

        self.declare_parameter("rgbd_publish_rate_hz", 30.0)
        self.declare_parameter("imu_publish_rate_hz", 100.0)

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

        self.rgbd_publish_rate_hz = self._positive_rate("rgbd_publish_rate_hz")
        self.imu_publish_rate_hz = self._positive_rate("imu_publish_rate_hz")

        self.latest_rgb = None
        self.latest_rgb_camera_info = None
        self.latest_depth = None
        self.latest_depth_camera_info = None
        self.latest_imu = None
        self._subscriptions = []
        self._timers = []

        self.rgb_pub = self.create_publisher(
            Image,
            self.get_parameter("rgb_topic").value,
            qos_profile_sensor_data,
        )
        self.rgb_camera_info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter("rgb_camera_info_topic").value,
            qos_profile_sensor_data,
        )
        self.depth_pub = self.create_publisher(
            Image,
            self.get_parameter("depth_topic").value,
            qos_profile_sensor_data,
        )
        self.depth_camera_info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter("depth_camera_info_topic").value,
            qos_profile_sensor_data,
        )
        self.imu_pub = self.create_publisher(
            Imu,
            self.get_parameter("imu_topic").value,
            qos_profile_sensor_data,
        )

        self._subscriptions.extend([
            self.create_subscription(
                Image,
                self.get_parameter("source_rgb_topic").value,
                self._store_rgb,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo,
                self.get_parameter("source_rgb_camera_info_topic").value,
                self._store_rgb_camera_info,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image,
                self.get_parameter("source_depth_topic").value,
                self._store_depth,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo,
                self.get_parameter("source_depth_camera_info_topic").value,
                self._store_depth_camera_info,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Imu,
                self.get_parameter("source_imu_topic").value,
                self._store_imu,
                qos_profile_sensor_data,
            ),
        ])

        self._timers.extend([
            self.create_timer(1.0 / self.rgbd_publish_rate_hz, self._publish_rgbd),
            self.create_timer(1.0 / self.imu_publish_rate_hz, self._publish_imu),
            self.create_timer(5.0, self._log_waiting_sources),
        ])

        self.get_logger().info(
            "Gemini 336 bridge publishing RGB-D at "
            f"{self.rgbd_publish_rate_hz:.1f} Hz and IMU at "
            f"{self.imu_publish_rate_hz:.1f} Hz"
        )

    def _positive_rate(self, parameter_name):
        rate_hz = float(self.get_parameter(parameter_name).value)
        if rate_hz <= 0.0:
            raise ValueError(f"{parameter_name} must be greater than 0 Hz")
        return rate_hz

    def _store_rgb(self, msg):
        self.latest_rgb = msg

    def _store_rgb_camera_info(self, msg):
        self.latest_rgb_camera_info = msg

    def _store_depth(self, msg):
        self.latest_depth = msg

    def _store_depth_camera_info(self, msg):
        self.latest_depth_camera_info = msg

    def _store_imu(self, msg):
        self.latest_imu = msg

    def _publish_rgbd(self):
        if self.latest_rgb is not None:
            self.rgb_pub.publish(self.latest_rgb)
        if self.latest_rgb_camera_info is not None:
            self.rgb_camera_info_pub.publish(self.latest_rgb_camera_info)
        if self.latest_depth is not None:
            self.depth_pub.publish(self.latest_depth)
        if self.latest_depth_camera_info is not None:
            self.depth_camera_info_pub.publish(self.latest_depth_camera_info)

    def _publish_imu(self):
        if self.latest_imu is not None:
            self.imu_pub.publish(self.latest_imu)

    def _log_waiting_sources(self):
        missing_sources = []
        if self.latest_rgb is None:
            missing_sources.append(self.get_parameter("source_rgb_topic").value)
        if self.latest_depth is None:
            missing_sources.append(self.get_parameter("source_depth_topic").value)
        if self.latest_imu is None:
            missing_sources.append(self.get_parameter("source_imu_topic").value)

        if missing_sources:
            source_status = "; ".join(
                self._describe_source_topic(topic) for topic in missing_sources
            )
            self.get_logger().warn(
                "Waiting for Gemini 336 source topics: " + source_status
            )

    def _describe_source_topic(self, topic):
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
