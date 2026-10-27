"""Publish the calculated track as Foxglove-compatible image annotations."""

from collections import deque
from dataclasses import dataclass
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import ImageMarker

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.pose_history import (
    KINEMATIC_STATE_TOPIC,
    PoseHistory,
    joint_state_pose,
    stamp_ns,
)
from legwheel_kinematics.projection import CameraCalibration
from legwheel_kinematics.track import project_track


RGB_IMAGE_TOPIC = "/legwheel_rgbd/color/image_raw"
RGB_INFO_TOPIC = "/legwheel_rgbd/color/camera_info"
DEPTH_IMAGE_TOPIC = "/legwheel_rgbd/depth/image_raw"
DEPTH_INFO_TOPIC = "/legwheel_rgbd/depth/camera_info"


@dataclass(frozen=True)
class PendingImage:
    """Keep annotation inputs while waiting for a delayed IMU pose bracket.

    Image payloads are deliberately excluded: only the original header and the
    calibration active at acquisition are needed to project the track.
    """

    header: object
    calibration: CameraCalibration
    publisher: object
    marker_id: int


class TrackOverlayNode(Node):
    """Draw the visible circular track over each raw camera stream."""

    def __init__(self) -> None:
        super().__init__("track_overlay")
        self.declare_parameter("experiment_config", "")
        self.declare_parameter("track_point_count", 360)

        config_path = self.get_parameter("experiment_config").value
        if not config_path:
            raise ValueError(
                "The experiment_config parameter must name an experiment YAML file."
            )
        self._config = KinematicsConfig.from_yaml(Path(config_path))
        self._track_point_count = int(self.get_parameter("track_point_count").value)

        self._pose_history = PoseHistory(maximum_gap_s=0.05)
        # At typical camera rates this holds several seconds of headers. A
        # missing publisher can never cause unbounded image-memory growth.
        self._pending_images = deque(maxlen=120)
        self._last_clock_ns = None
        self._using_sim_time = bool(self.get_parameter("use_sim_time").value)
        self._rgb_calibration = None
        self._depth_calibration = None
        self._invalid_state_logged = False

        self._rgb_marker_publisher = self.create_publisher(
            ImageMarker,
            "/legwheel/visualization/rgb_track",
            10,
        )
        self._depth_marker_publisher = self.create_publisher(
            ImageMarker,
            "/legwheel/visualization/depth_track",
            10,
        )

        self.create_subscription(
            JointState,
            KINEMATIC_STATE_TOPIC,
            self._kinematic_state_callback,
            # Rejected samples are explicit barriers. Reliable transport keeps
            # a dropped barrier from looking like an ordinary short interval.
            QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            Clock,
            "/clock",
            self._clock_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            RGB_INFO_TOPIC,
            self._rgb_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            DEPTH_INFO_TOPIC,
            self._depth_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            RGB_IMAGE_TOPIC,
            self._rgb_image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            DEPTH_IMAGE_TOPIC,
            self._depth_image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Track overlay is waiting for reliable IMU poses on "
            f"{KINEMATIC_STATE_TOPIC}. Start kinematic_joint_state_node when "
            "running this overlay directly."
        )

    def _clock_callback(self, message: Clock) -> None:
        """Do not associate images with poses from before a backward bag seek."""
        if not self._using_sim_time:
            return
        clock_ns = stamp_ns(message.clock)
        if self._last_clock_ns is not None and clock_ns < self._last_clock_ns:
            self._pose_history.clear()
            self._pending_images.clear()
            self._rgb_calibration = None
            self._depth_calibration = None
        self._last_clock_ns = clock_ns

    def _in_playback_epoch(self, stamp) -> bool:
        """Discard queued messages from the future epoch after a backward seek."""
        # The IMU publisher deliberately lags playback for smoothing. A pose
        # well ahead of /clock therefore belongs to the old playback queue.
        # A small tolerance permits ordinary /clock callback scheduling jitter.
        return (
            self._last_clock_ns is None
            or stamp_ns(stamp) <= self._last_clock_ns + 1_000_000_000
        )

    def _kinematic_state_callback(self, message: JointState) -> None:
        """Buffer source-timestamped IMU poses, including rejection barriers."""
        if not self._in_playback_epoch(message.header.stamp):
            return
        timestamp_ns = stamp_ns(message.header.stamp)
        try:
            yaw, pitch = joint_state_pose(message)
        except ValueError as error:
            # Empty JointStates are the shared publisher's explicit invalid
            # samples. Retaining them prevents interpolation through impacts,
            # even when the surrounding valid samples are less than 50 ms apart.
            self._pose_history.append_unavailable(timestamp_ns)
            if message.name and not self._invalid_state_logged:
                self.get_logger().warning(f"Cannot use IMU kinematic state: {error}")
                self._invalid_state_logged = True
        else:
            self._pose_history.append(timestamp_ns, yaw, pitch)
        self._flush_pending_images()

    def _rgb_info_callback(self, message: CameraInfo) -> None:
        if self._in_playback_epoch(message.header.stamp):
            self._rgb_calibration = CameraCalibration.from_message(message)

    def _depth_info_callback(self, message: CameraInfo) -> None:
        if self._in_playback_epoch(message.header.stamp):
            self._depth_calibration = CameraCalibration.from_message(message)

    def _rgb_image_callback(self, message: Image) -> None:
        self._queue_image(
            message,
            self._rgb_calibration,
            self._rgb_marker_publisher,
            marker_id=0,
        )

    def _depth_image_callback(self, message: Image) -> None:
        self._queue_image(
            message,
            self._depth_calibration,
            self._depth_marker_publisher,
            marker_id=1,
        )

    def _queue_image(
        self,
        image: Image,
        calibration: CameraCalibration | None,
        publisher,
        marker_id: int,
    ) -> None:
        """Wait for bracketing poses without retaining the image pixel array."""
        if calibration is None or not self._in_playback_epoch(image.header.stamp):
            return
        self._pending_images.append(
            PendingImage(image.header, calibration, publisher, marker_id)
        )
        self._flush_pending_images()

    def _flush_pending_images(self) -> None:
        """Publish supported images and discard gaps; future images keep waiting."""
        newest_stamp_ns = self._pose_history.newest_stamp_ns
        if newest_stamp_ns is None:
            return
        waiting = deque(maxlen=self._pending_images.maxlen)
        while self._pending_images:
            pending = self._pending_images.popleft()
            image_stamp_ns = stamp_ns(pending.header.stamp)
            if image_stamp_ns > newest_stamp_ns:
                waiting.append(pending)
                continue
            try:
                _, theta_p = self._pose_history.sample(image_stamp_ns)
            except ValueError:
                # A pose after this image has already arrived. Missing older
                # coverage or an impact barrier cannot be repaired by using the
                # latest angle, so this image deliberately has no annotation.
                continue
            self._publish_marker(pending, theta_p)
        self._pending_images = waiting

    def _publish_marker(self, pending: PendingImage, theta_p: float) -> None:
        """Project using IMU beam pitch at this image's actual acquisition time."""

        try:
            # Circular track geometry is invariant to central yaw. Pitch is
            # already corrected for camera_front_angle_deg by the estimator;
            # projection retains the same physical camera mounting transform.
            projection = project_track(
                self._config,
                theta_p,
                pending.calibration,
                point_count=self._track_point_count,
            )
        except ValueError as error:
            self.get_logger().error(f"Cannot calculate track overlay: {error}")
            return

        if projection.polygon_pixels.size == 0:
            return

        marker = ImageMarker()
        # Exact image timestamps allow Foxglove's synchronized display to match
        # an annotation with the image for which it was calculated.
        marker.header = pending.header
        marker.ns = "legwheel_track"
        marker.id = pending.marker_id
        marker.type = ImageMarker.LINE_STRIP
        marker.action = ImageMarker.ADD
        marker.scale = 2.0
        marker.outline_color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)

        closed_polygon = list(projection.polygon_pixels)
        closed_polygon.append(projection.polygon_pixels[0])
        for u, v in closed_polygon:
            marker.points.append(Point(x=float(u), y=float(v), z=0.0))
        pending.publisher.publish(marker)


def main(args=None) -> None:
    """Run the track-overlay ROS node."""
    rclpy.init(args=args)
    node = TrackOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
