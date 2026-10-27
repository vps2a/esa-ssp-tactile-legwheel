"""Publish replay-time kinematic frames and 3D track points for Foxglove."""

from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.pose_history import (
    KINEMATIC_STATE_TOPIC,
    joint_state_pose,
    stamp_ns,
)
from legwheel_kinematics.track import generate_track_points
from legwheel_kinematics.transforms import (
    KINEMATIC_FRAME_IDS,
    frame_coordinates,
    rotation_matrix_to_quaternion,
    transformation_matrices_from_config,
)


VISUALIZATION_TOPIC = "/legwheel/visualization/kinematics_3d"


def _point(values) -> Point:
    """Create a ROS point from three numerical values."""
    return Point(
        x=float(values[0]),
        y=float(values[1]),
        z=float(values[2]),
    )


def _color(red: float, green: float, blue: float, alpha: float = 1.0):
    return ColorRGBA(r=red, g=green, b=blue, a=alpha)


def _transform_message(parent_frame, child_frame, matrix, stamp):
    """Convert one notebook transformation matrix to TransformStamped."""
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent_frame
    message.child_frame_id = child_frame
    message.transform.translation.x = float(matrix[0, 3])
    message.transform.translation.y = float(matrix[1, 3])
    message.transform.translation.z = float(matrix[2, 3])
    x, y, z, w = rotation_matrix_to_quaternion(matrix[:3, :3])
    message.transform.rotation.x = x
    message.transform.rotation.y = y
    message.transform.rotation.z = z
    message.transform.rotation.w = w
    return message


class KinematicsVisualizationNode(Node):
    """Recalculate and publish notebook geometry during rosbag playback."""

    def __init__(self) -> None:
        super().__init__("kinematics_visualization")
        self.declare_parameter("experiment_config", "")
        self.declare_parameter("track_point_count", 360)
        self.declare_parameter("publish_tf", True)

        config_path = self.get_parameter("experiment_config").value
        if not config_path:
            raise ValueError(
                "The experiment_config parameter must name an experiment YAML file."
            )
        self._config = KinematicsConfig.from_yaml(Path(config_path))
        self._track_points = generate_track_points(
            self._config,
            int(self.get_parameter("track_point_count").value),
        )
        self._invalid_state_logged = False
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._using_sim_time = bool(self.get_parameter("use_sim_time").value)
        self._last_clock_ns = None

        self._transform_broadcaster = TransformBroadcaster(self)
        self._static_transform_broadcaster = StaticTransformBroadcaster(self)
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            VISUALIZATION_TOPIC,
            10,
        )
        self.create_subscription(
            JointState,
            KINEMATIC_STATE_TOPIC,
            self._kinematic_state_callback,
            QoSProfile(depth=200),
        )
        self.create_subscription(
            Clock,
            "/clock",
            self._clock_callback,
            qos_profile_sensor_data,
        )

        if self._publish_tf:
            self._publish_static_transforms()
        self.get_logger().info(
            "Replay visualization is waiting for reliable IMU poses on "
            f"{KINEMATIC_STATE_TOPIC}. Start kinematic_joint_state_node when "
            "running this visualizer directly."
        )

    def _publish_static_transforms(self) -> None:
        """Publish only the three fixed beam-end and camera mounting transforms."""
        _, _, T23, T34, T45 = transformation_matrices_from_config(
            self._config,
            theta_p=0.0,
        )
        stamp = self.get_clock().now().to_msg()
        transforms = [
            _transform_message(
                KINEMATIC_FRAME_IDS[2], KINEMATIC_FRAME_IDS[3], T23, stamp
            ),
            _transform_message(
                KINEMATIC_FRAME_IDS[3], KINEMATIC_FRAME_IDS[4], T34, stamp
            ),
            _transform_message(
                KINEMATIC_FRAME_IDS[4], KINEMATIC_FRAME_IDS[5], T45, stamp
            ),
        ]
        self._static_transform_broadcaster.sendTransform(transforms)

    def _clock_callback(self, message: Clock) -> None:
        """Track the current playback epoch independently of delayed pose stamps."""
        if self._using_sim_time:
            self._last_clock_ns = stamp_ns(message.clock)

    def _kinematic_state_callback(self, message: JointState) -> None:
        """Publish measured yaw and IMU pitch at the original sensor timestamp."""
        if (
            self._last_clock_ns is not None
            and stamp_ns(message.header.stamp) > self._last_clock_ns + 1_000_000_000
        ):
            # A deep DDS queue can still contain poses from before a rewind.
            # Do not republish those future-epoch transforms after the seek.
            return
        # Empty derived states mark rejected IMU samples. Do not turn them into
        # either a knee-based pose or an invalid TF transform.
        if not message.name and not message.position:
            return
        try:
            theta_y, theta_p = joint_state_pose(message)
        except ValueError as error:
            if not self._invalid_state_logged:
                self.get_logger().error(f"Cannot publish replay visualization: {error}")
                self._invalid_state_logged = True
            return

        T01, T12, _, _, _ = transformation_matrices_from_config(
            self._config,
            theta_p,
            theta_y=theta_y,
        )
        # Yaw now comes from the central encoder, so both pivot transforms
        # are dynamic. Publishing T01 as static would rotate the markers but
        # leave downstream TF consumers at the old, zero-yaw camera pose.
        if self._publish_tf:
            self._transform_broadcaster.sendTransform(
                [
                    _transform_message(
                        KINEMATIC_FRAME_IDS[0],
                        KINEMATIC_FRAME_IDS[1],
                        T01,
                        message.header.stamp,
                    ),
                    _transform_message(
                        KINEMATIC_FRAME_IDS[1],
                        KINEMATIC_FRAME_IDS[2],
                        T12,
                        message.header.stamp,
                    ),
                ]
            )

        Frame_coords = frame_coordinates(self._config, theta_p, theta_y=theta_y)
        self._marker_publisher.publish(
            self._make_marker_array(Frame_coords, message.header.stamp)
        )

    def _make_marker_array(self, Frame_coords, stamp) -> MarkerArray:
        """Create track-point, frame-origin and kinematic-chain markers."""
        markers = MarkerArray()
        markers.markers.extend(
            [
                self._track_marker(
                    marker_id=0,
                    name="inner_track_points",
                    points=self._track_points.inner_track_points,
                    color=_color(0.1, 0.8, 1.0),
                    stamp=stamp,
                ),
                self._track_marker(
                    marker_id=1,
                    name="outer_track_points",
                    points=self._track_points.outer_track_points,
                    color=_color(1.0, 0.5, 0.1),
                    stamp=stamp,
                ),
            ]
        )

        origins = np.vstack([matrix[:3, 3] for matrix in Frame_coords])
        origin_marker = self._base_marker(2, "frame_origins", stamp)
        origin_marker.type = Marker.SPHERE_LIST
        origin_marker.scale.x = 0.025
        origin_marker.scale.y = 0.025
        origin_marker.scale.z = 0.025
        origin_marker.color = _color(1.0, 1.0, 1.0)
        origin_marker.points = [_point(origin) for origin in origins]

        chain_marker = self._base_marker(3, "kinematic_chain", stamp)
        chain_marker.type = Marker.LINE_STRIP
        chain_marker.scale.x = 0.008
        chain_marker.color = _color(0.9, 0.9, 0.1)
        chain_marker.points = [_point(origin) for origin in origins]
        markers.markers.extend([origin_marker, chain_marker])
        return markers

    def _track_marker(
        self,
        marker_id: int,
        name: str,
        points: np.ndarray,
        color: ColorRGBA,
        stamp,
    ) -> Marker:
        marker = self._base_marker(marker_id, name, stamp)
        marker.type = Marker.POINTS
        marker.scale.x = 0.012
        marker.scale.y = 0.012
        marker.color = color
        marker.points = [_point(point) for point in points]
        return marker

    @staticmethod
    def _base_marker(marker_id: int, name: str, stamp) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = KINEMATIC_FRAME_IDS[0]
        marker.ns = name
        marker.id = marker_id
        marker.action = Marker.ADD
        # A valid identity pose is required even though points are already in
        # legwheel_base coordinates.
        marker.pose.orientation.w = 1.0
        return marker


def main(args=None) -> None:
    """Run the replay-only kinematics visualization node."""
    rclpy.init(args=args)
    node = KinematicsVisualizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
