#!/usr/bin/env python3

"""Publish a synchronized, project-stable view of the Gemini 336 streams."""

from message_filters import ApproximateTimeSynchronizer, Subscriber
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

from legwheel_rgbd.stream_utils import (
    camera_info_signature,
    stamp_nanoseconds,
    stamps_strictly_advance,
)


class Gemini336CameraNode(Node):
    """Bridge Orbbec topics without manufacturing duplicate sensor samples."""

    def __init__(self):
        super().__init__("gemini_336_camera_node")

        self.declare_parameter("rgbd_sync_queue_size", 30)
        self.declare_parameter("rgbd_sync_slop_s", 0.01)

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

        sync_queue_size = int(self.get_parameter("rgbd_sync_queue_size").value)
        sync_slop_s = float(self.get_parameter("rgbd_sync_slop_s").value)
        if sync_queue_size < 2:
            raise ValueError("rgbd_sync_queue_size must be at least 2")
        if sync_slop_s <= 0.0:
            raise ValueError("rgbd_sync_slop_s must be greater than 0 seconds")

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

        # message_filters consumes each source image at most once. Images are
        # forwarded only from the paired callback, so the project topics always
        # contain RGB and aligned depth observations from the same time window.
        self._rgb_subscriber = Subscriber(
            self,
            Image,
            self._source_topics["rgb"],
            qos_profile=qos_profile_sensor_data,
        )
        self._depth_subscriber = Subscriber(
            self,
            Image,
            self._source_topics["depth"],
            qos_profile=qos_profile_sensor_data,
        )
        self._rgb_subscriber.registerCallback(self._mark_rgb_received)
        self._depth_subscriber.registerCallback(self._mark_depth_received)

        self._rgbd_synchronizer = ApproximateTimeSynchronizer(
            [self._rgb_subscriber, self._depth_subscriber],
            queue_size=sync_queue_size,
            slop=sync_slop_s,
        )
        self._rgbd_synchronizer.registerCallback(self._publish_rgbd_pair)

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

        self._last_rgb_stamp_ns = None
        self._last_depth_stamp_ns = None
        self._last_imu_stamp_ns = None
        self._rgb_camera_info_signature = None
        self._depth_camera_info_signature = None
        self._synchronized_pair_count = 0
        self._warned_nonadvancing_streams = set()

        self._waiting_log_timer = self.create_timer(5.0, self._log_waiting_sources)

        self.get_logger().info(
            "Gemini 336 bridge forwarding synchronized RGB-D pairs with "
            f"a {sync_slop_s * 1000.0:.1f} ms tolerance; IMU forwarding is event-driven"
        )

    def _mark_rgb_received(self, _message):
        """Track source discovery separately from successful synchronization."""
        self._source_received["rgb"] = True

    def _mark_depth_received(self, _message):
        """Track source discovery separately from successful synchronization."""
        self._source_received["depth"] = True

    def _publish_rgbd_pair(self, rgb_message, depth_message):
        """Forward one strictly advancing RGB/depth pair exactly once."""
        rgb_stamp_ns = stamp_nanoseconds(rgb_message.header.stamp)
        depth_stamp_ns = stamp_nanoseconds(depth_message.header.stamp)

        if not self._stamps_advance(
            "RGB-D",
            (rgb_stamp_ns, depth_stamp_ns),
            (self._last_rgb_stamp_ns, self._last_depth_stamp_ns),
        ):
            return

        self._last_rgb_stamp_ns = rgb_stamp_ns
        self._last_depth_stamp_ns = depth_stamp_ns
        self._synchronized_pair_count += 1
        self.rgb_pub.publish(rgb_message)
        self.depth_pub.publish(depth_message)

    def _publish_rgb_camera_info_if_changed(self, message):
        """Publish RGB calibration initially and whenever its values change."""
        self._source_received["rgb_info"] = True
        signature = camera_info_signature(message)
        if signature == self._rgb_camera_info_signature:
            return
        self._rgb_camera_info_signature = signature
        self.rgb_camera_info_pub.publish(message)

    def _publish_depth_camera_info_if_changed(self, message):
        """Publish depth calibration initially and whenever its values change."""
        self._source_received["depth_info"] = True
        signature = camera_info_signature(message)
        if signature == self._depth_camera_info_signature:
            return
        self._depth_camera_info_signature = signature
        self.depth_camera_info_pub.publish(message)

    def _publish_imu(self, message):
        """Forward each new IMU sample directly instead of replaying cached data."""
        self._source_received["imu"] = True
        stamp_ns = stamp_nanoseconds(message.header.stamp)
        if not self._stamps_advance(
            "IMU",
            (stamp_ns,),
            (self._last_imu_stamp_ns,),
        ):
            return
        self._last_imu_stamp_ns = stamp_ns
        self.imu_pub.publish(message)

    def _stamps_advance(self, stream_name, new_stamps, previous_stamps):
        """Reject zero, repeated, or regressing stamps without flooding logs."""
        valid = stamps_strictly_advance(new_stamps, previous_stamps)
        if valid:
            return True

        if stream_name not in self._warned_nonadvancing_streams:
            self._warned_nonadvancing_streams.add(stream_name)
            self.get_logger().warning(
                f"Dropping {stream_name} sample with a zero, repeated, or "
                "regressing sensor timestamp. This warning is emitted once per stream."
            )
        return False

    def _log_waiting_sources(self):
        """Report missing publishers or a synchronization configuration problem."""
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
            return

        if self._synchronized_pair_count == 0:
            self.get_logger().warning(
                "RGB and depth are arriving, but no synchronized pair has met "
                "rgbd_sync_slop_s. Check camera frame synchronization and timestamps."
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
