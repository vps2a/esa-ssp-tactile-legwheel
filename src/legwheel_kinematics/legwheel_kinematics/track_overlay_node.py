"""Publish the calculated track as Foxglove-compatible image annotations."""

from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import ImageMarker

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.projection import CameraCalibration
from legwheel_kinematics.track import project_track
from legwheel_kinematics.transforms import theta_p_from_config


RGB_IMAGE_TOPIC = "/legwheel_rgbd/color/image_raw"
RGB_INFO_TOPIC = "/legwheel_rgbd/color/camera_info"
DEPTH_IMAGE_TOPIC = "/legwheel_rgbd/depth/image_raw"
DEPTH_INFO_TOPIC = "/legwheel_rgbd/depth/camera_info"
LEG_STATE_TOPIC = "/legwheel/joint_states"
KNEE_JOINT_NAME = "knee_joint"


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
        self._track_point_count = int(
            self.get_parameter("track_point_count").value
        )

        self._knee_joint = None
        self._rgb_calibration = None
        self._depth_calibration = None
        self._waiting_for_state_logged = False

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
            LEG_STATE_TOPIC,
            self._leg_state_callback,
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
        self.get_logger().info("Track overlay node initialized.")

    def _leg_state_callback(self, message: JointState) -> None:
        """Keep the latest named knee angle for the visualization."""
        try:
            knee_index = list(message.name).index(KNEE_JOINT_NAME)
            self._knee_joint = float(message.position[knee_index])
        except (ValueError, IndexError):
            if not self._waiting_for_state_logged:
                self.get_logger().warning(
                    "JointState does not contain a knee_joint position."
                )
                self._waiting_for_state_logged = True

    def _rgb_info_callback(self, message: CameraInfo) -> None:
        self._rgb_calibration = CameraCalibration.from_message(message)

    def _depth_info_callback(self, message: CameraInfo) -> None:
        self._depth_calibration = CameraCalibration.from_message(message)

    def _rgb_image_callback(self, message: Image) -> None:
        self._publish_marker(
            message,
            self._rgb_calibration,
            self._rgb_marker_publisher,
            marker_id=0,
        )

    def _depth_image_callback(self, message: Image) -> None:
        self._publish_marker(
            message,
            self._depth_calibration,
            self._depth_marker_publisher,
            marker_id=1,
        )

    def _publish_marker(
        self,
        image: Image,
        calibration: CameraCalibration | None,
        publisher,
        marker_id: int,
    ) -> None:
        if self._knee_joint is None or calibration is None:
            return

        try:
            theta_p = theta_p_from_config(self._knee_joint, self._config)
            projection = project_track(
                self._config,
                theta_p,
                calibration,
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
        marker.header = image.header
        marker.ns = "legwheel_track"
        marker.id = marker_id
        marker.type = ImageMarker.LINE_STRIP
        marker.action = ImageMarker.ADD
        marker.scale = 2.0
        marker.outline_color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)

        closed_polygon = list(projection.polygon_pixels)
        closed_polygon.append(projection.polygon_pixels[0])
        for u, v in closed_polygon:
            marker.points.append(Point(x=float(u), y=float(v), z=0.0))
        publisher.publish(marker)


def main(args=None) -> None:
    """Run the track-overlay ROS node."""
    rclpy.init(args=args)
    node = TrackOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
