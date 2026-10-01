"""Foxglove visualization of derived packets synchronized to rosbag /clock."""

import argparse
from bisect import bisect_right
import json
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import ColorRGBA, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import ImageMarker, Marker, MarkerArray

from legwheel_dataset.frame_playback import FramePose, interpolate_frame_pose
from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.track import generate_track_points
from legwheel_kinematics.transforms import (
    individual_transformation_matrices,
    rotation_matrix_to_quaternion,
)


RGB_ANNOTATION_TOPIC = "/legwheel/dataset/rgb_annotations"
DEPTH_ANNOTATION_TOPIC = "/legwheel/dataset/depth_annotations"
MARKER_TOPIC = "/legwheel/dataset/geometry"
SLOPE_TEXT_TOPIC = "/legwheel/dataset/slope_text"


def _stamp(timestamp_ns: int) -> Time:
    message = Time()
    message.sec = int(timestamp_ns // 1_000_000_000)
    message.nanosec = int(timestamp_ns % 1_000_000_000)
    return message


def _point(values) -> Point:
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def _color(red: float, green: float, blue: float, alpha: float = 1.0):
    return ColorRGBA(r=red, g=green, b=blue, a=alpha)


def _transform(parent: str, child: str, matrix: np.ndarray, stamp: Time):
    message = TransformStamped()
    message.header.frame_id = parent
    message.header.stamp = stamp
    message.child_frame_id = child
    message.transform.translation.x = float(matrix[0, 3])
    message.transform.translation.y = float(matrix[1, 3])
    message.transform.translation.z = float(matrix[2, 3])
    x, y, z, w = rotation_matrix_to_quaternion(matrix[:3, :3])
    message.transform.rotation.x = x
    message.transform.rotation.y = y
    message.transform.rotation.z = z
    message.transform.rotation.w = w
    return message


class PacketVisualizationNode(Node):
    """Publish a packet when bag playback clock reaches its image timestamp."""

    def __init__(self, dataset_directory: Path) -> None:
        super().__init__("legwheel_dataset_visualization")
        self._dataset_directory = dataset_directory.resolve()
        provenance = json.loads(
            (self._dataset_directory / "provenance.json").read_text(
                encoding="utf-8"
            )
        )
        self._kinematics_config = KinematicsConfig.from_experiment_dict(
            provenance["experiment_config"]
        )
        point_count = int(provenance["processing_config"]["track_point_count"])
        self._full_track = generate_track_points(
            self._kinematics_config, point_count
        )

        self._records = []
        with (self._dataset_directory / "manifest.jsonl").open(
            "r", encoding="utf-8"
        ) as manifest:
            for line in manifest:
                if line.strip():
                    self._records.append(json.loads(line))
        self._records.sort(key=lambda value: int(value["image_time_ns"]))
        self._times = [int(record["image_time_ns"]) for record in self._records]
        if not self._records:
            raise ValueError("The dataset manifest contains no packets to visualize.")

        # Packet overlays remain discrete, but TF must be valid at the current
        # /clock time. Preload the saved poses once so each clock callback can
        # cheaply interpolate a smooth, correctly timestamped frame chain.
        self._frame_poses = []
        for record in self._records:
            packet_directory = self._dataset_directory / record["packet_path"]
            metadata = json.loads(
                (packet_directory / "metadata.json").read_text(encoding="utf-8")
            )
            self._frame_poses.append(
                FramePose(
                    timestamp_ns=int(metadata["image_time_ns"]),
                    theta_y_rad=float(metadata["theta_y_image_kinematic_rad"]),
                    theta_p_rad=float(metadata["theta_p_rgb_rad"]),
                )
            )
        for previous, current in zip(self._frame_poses, self._frame_poses[1:]):
            if current.timestamp_ns <= previous.timestamp_ns:
                raise ValueError(
                    "Dataset packet timestamps must be strictly increasing."
                )
        self._next_index = 0
        self._last_clock_ns = None
        self._last_clock_region = None

        self._rgb_publisher = self.create_publisher(
            ImageMarker, RGB_ANNOTATION_TOPIC, 20
        )
        self._depth_publisher = self.create_publisher(
            ImageMarker, DEPTH_ANNOTATION_TOPIC, 20
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, MARKER_TOPIC, 10
        )
        self._slope_text_publisher = self.create_publisher(
            String, SLOPE_TEXT_TOPIC, 10
        )
        self._transform_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Clock,
            "/clock",
            self._clock_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Loaded {len(self._records)} derived packets; waiting for /clock. "
            "Dataset TF will be interpolated at every clock update. "
            f"Moving-frame interval: {self._times[0]} to {self._times[-1]} ns "
            f"({(self._times[-1] - self._times[0]) / 1e9:.3f} s)."
        )

    def _clock_callback(self, message: Clock) -> None:
        clock_ns = int(message.clock.sec) * 1_000_000_000 + int(
            message.clock.nanosec
        )
        if self._last_clock_ns is None or clock_ns < self._last_clock_ns:
            # Seeking or looping a bag must make earlier packets publish again.
            self._next_index = bisect_right(self._times, clock_ns)
            if self._next_index > 0 and (
                self._last_clock_ns is None or clock_ns < self._last_clock_ns
            ):
                self._next_index -= 1
        self._last_clock_ns = clock_ns

        # Publish TF at the exact current simulated time. Previously TF was
        # emitted only at sparse packet timestamps; Foxglove could then have no
        # transform valid for its current render time and show a frozen chain.
        frame_pose = interpolate_frame_pose(self._frame_poses, clock_ns)
        if frame_pose is not None:
            if self._last_clock_region != "inside":
                self.get_logger().info(
                    "Playback entered the dataset moving-frame interval."
                )
            self._last_clock_region = "inside"
            self._publish_transforms(frame_pose)
        else:
            clock_region = "before" if clock_ns < self._times[0] else "after"
            if self._last_clock_region != clock_region:
                self.get_logger().warning(
                    "Playback is "
                    f"{clock_region} the dataset moving-frame interval; "
                    "seek into the analysed steady-motion interval to see "
                    "the dataset_* frames move."
                )
            self._last_clock_region = clock_region

        while (
            self._next_index < len(self._records)
            and self._times[self._next_index] <= clock_ns
        ):
            self._publish_packet(self._records[self._next_index])
            self._next_index += 1

    def _publish_packet(self, record: dict) -> None:
        packet_directory = self._dataset_directory / record["packet_path"]
        metadata = json.loads(
            (packet_directory / "metadata.json").read_text(encoding="utf-8")
        )
        geometry = np.load(
            packet_directory / "geometry.npz", allow_pickle=False
        )
        self._publish_image_markers(metadata, geometry, rgb=True)
        self._publish_image_markers(metadata, geometry, rgb=False)
        self._publish_3d_markers(packet_directory, metadata, geometry)

    def _publish_image_markers(self, metadata, geometry, *, rgb: bool) -> None:
        prefix = "rgb" if rgb else "depth"
        publisher = self._rgb_publisher if rgb else self._depth_publisher
        timestamp_ns = int(metadata[f"{prefix}_time_ns"])
        frame_id = metadata[f"{prefix}_frame_id"]
        stamp = _stamp(timestamp_ns)

        patch = np.asarray(geometry[f"{prefix}_patch_polygon_pixels"])
        patch_marker = self._image_marker(
            stamp, frame_id, marker_id=0, name="selected_patch"
        )
        patch_marker.type = ImageMarker.LINE_STRIP
        patch_marker.outline_color = _color(1.0, 0.1, 0.1)
        patch_marker.points = [_point((u, v, 0.0)) for u, v in patch]
        patch_marker.points.append(_point((patch[0, 0], patch[0, 1], 0.0)))
        publisher.publish(patch_marker)

        for marker_id, boundary_name, color in (
            (1, "inner_track", _color(0.1, 0.8, 1.0)),
            (2, "outer_track", _color(1.0, 0.6, 0.1)),
        ):
            pixels = np.asarray(geometry[f"{prefix}_{boundary_name}_pixels"])
            valid = np.asarray(geometry[f"{prefix}_{boundary_name}_valid"])
            marker = self._image_marker(
                stamp, frame_id, marker_id=marker_id, name=boundary_name
            )
            marker.type = ImageMarker.POINTS
            marker.outline_color = color
            marker.points = [
                _point((u, v, 0.0)) for u, v in pixels[valid]
            ]
            publisher.publish(marker)

    @staticmethod
    def _image_marker(stamp, frame_id, marker_id: int, name: str):
        marker = ImageMarker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = name
        marker.id = marker_id
        marker.action = ImageMarker.ADD
        marker.scale = 2.0
        return marker

    def _publish_transforms(self, frame_pose: FramePose) -> None:
        """Publish the interpolated camera chain at the current bag time."""
        matrices = individual_transformation_matrices(
            d1=self._kinematics_config.d1,
            theta_y=frame_pose.theta_y_rad,
            theta_p=frame_pose.theta_p_rad,
            a_b=self._kinematics_config.a_b,
            h_b=self._kinematics_config.h_b,
            camera_beam_offset=self._kinematics_config.camera_beam_offset,
            camera_height=self._kinematics_config.camera_height,
            camera_offset_from_beam_centre=(
                self._kinematics_config.camera_offset_from_beam_centre
            ),
            camera_angle_deg=self._kinematics_config.camera_angle_deg,
        )
        frame_ids = (
            "legwheel_base",
            "dataset_theta_y_link",
            "dataset_theta_p_link",
            "dataset_beam_end_link",
            "dataset_camera_mount_link",
            "dataset_camera_optical_frame",
        )
        stamp = _stamp(frame_pose.timestamp_ns)
        transforms = [
            _transform(frame_ids[index], frame_ids[index + 1], matrix, stamp)
            for index, matrix in enumerate(matrices)
        ]
        self._transform_broadcaster.sendTransform(transforms)

    def _publish_3d_markers(self, packet_directory, metadata, geometry) -> None:
        stamp = _stamp(int(metadata["image_time_ns"]))
        markers = MarkerArray()
        clear_previous = self._base_marker(0, "clear_previous_packet", stamp)
        clear_previous.action = Marker.DELETEALL
        markers.markers.append(clear_previous)
        markers.markers.extend(
            [
                self._points_marker(
                    0,
                    "inner_track",
                    self._full_track.inner_track_points,
                    _color(0.1, 0.8, 1.0),
                    stamp,
                    0.012,
                ),
                self._points_marker(
                    1,
                    "outer_track",
                    self._full_track.outer_track_points,
                    _color(1.0, 0.6, 0.1),
                    stamp,
                    0.012,
                ),
                self._line_marker(
                    2,
                    "patch_inner_boundary",
                    geometry["patch_inner_points_base_m"],
                    _color(1.0, 0.1, 0.1),
                    stamp,
                ),
                self._line_marker(
                    3,
                    "patch_outer_boundary",
                    geometry["patch_outer_points_base_m"],
                    _color(1.0, 0.1, 0.1),
                    stamp,
                ),
            ]
        )

        features_path = packet_directory / "stage2" / "features.json"
        point_map_path = packet_directory / "stage2" / "point_map.npz"
        if features_path.is_file() and point_map_path.is_file():
            features = json.loads(features_path.read_text(encoding="utf-8"))
            if features.get("valid"):
                point_map = np.load(point_map_path, allow_pickle=False)
                points = point_map["points_base_m"]
                inliers = point_map["ransac_inlier_mask"]
                markers.markers.append(
                    self._points_marker(
                        4,
                        "depth_plane_inliers",
                        points[inliers],
                        _color(0.2, 1.0, 0.2, 0.7),
                        stamp,
                        0.006,
                    )
                )
                centre = np.mean(points[inliers], axis=0)
                normal = np.asarray(features["plane_normal"], dtype=float)
                markers.markers.append(
                    self._arrow_marker(
                        5,
                        "plane_normal",
                        centre,
                        centre + normal * 0.15,
                        _color(0.8, 0.2, 1.0),
                        stamp,
                    )
                )
                angle = float(metadata["patch_centre_kinematic_angle_rad"])
                direction = float(metadata["travel_direction_kinematic"])
                radial_outward = np.asarray(
                    [np.cos(angle), np.sin(angle), 0.0], dtype=float
                )
                travel = direction * np.asarray(
                    [-np.sin(angle), np.cos(angle), 0.0], dtype=float
                )
                markers.markers.extend(
                    [
                        self._arrow_marker(
                            6,
                            "travel_direction",
                            centre,
                            centre + travel * 0.15,
                            _color(1.0, 0.2, 0.2),
                            stamp,
                        ),
                        self._arrow_marker(
                            7,
                            "radially_outward",
                            centre,
                            centre + radial_outward * 0.15,
                            _color(0.2, 0.5, 1.0),
                            stamp,
                        ),
                    ]
                )
                plane_outline = []
                plane_offset = float(features["plane_offset"])
                for tangent_scale, radial_scale in (
                    (-0.10, -0.10),
                    (0.10, -0.10),
                    (0.10, 0.10),
                    (-0.10, 0.10),
                    (-0.10, -0.10),
                ):
                    corner = (
                        centre
                        + travel * tangent_scale
                        + radial_outward * radial_scale
                    )
                    if abs(normal[2]) > 1e-9:
                        corner[2] = -(
                            normal[0] * corner[0]
                            + normal[1] * corner[1]
                            + plane_offset
                        ) / normal[2]
                    plane_outline.append(corner)
                markers.markers.append(
                    self._line_marker(
                        8,
                        "fitted_plane",
                        np.asarray(plane_outline),
                        _color(0.8, 0.2, 1.0),
                        stamp,
                    )
                )
                text = (
                    f"along={features['along_slope_deg']:.2f} deg, "
                    f"radial={features['cross_slope_deg']:.2f} deg"
                )
                markers.markers.append(
                    self._text_marker(9, "slope", centre, text, stamp)
                )
                self._slope_text_publisher.publish(String(data=text))
        self._marker_publisher.publish(markers)

    @staticmethod
    def _base_marker(marker_id: int, namespace: str, stamp: Time) -> Marker:
        marker = Marker()
        marker.header.frame_id = "legwheel_base"
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _points_marker(self, marker_id, namespace, points, color, stamp, scale):
        marker = self._base_marker(marker_id, namespace, stamp)
        marker.type = Marker.POINTS
        marker.scale.x = scale
        marker.scale.y = scale
        marker.color = color
        marker.points = [_point(point) for point in np.asarray(points)]
        return marker

    def _line_marker(self, marker_id, namespace, points, color, stamp):
        marker = self._base_marker(marker_id, namespace, stamp)
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.008
        marker.color = color
        marker.points = [_point(point) for point in np.asarray(points)]
        return marker

    def _arrow_marker(self, marker_id, namespace, start, end, color, stamp):
        marker = self._base_marker(marker_id, namespace, stamp)
        marker.type = Marker.ARROW
        marker.scale.x = 0.012
        marker.scale.y = 0.025
        marker.scale.z = 0.035
        marker.color = color
        marker.points = [_point(start), _point(end)]
        return marker

    def _text_marker(self, marker_id, namespace, position, text, stamp):
        marker = self._base_marker(marker_id, namespace, stamp)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.z = 0.04
        marker.color = _color(1.0, 1.0, 1.0)
        marker.pose.position = _point(np.asarray(position) + [0.0, 0.0, 0.08])
        marker.text = text
        return marker


def main(args=None) -> None:
    """Follow rosbag /clock and publish the matching derived packet overlays."""
    parser = argparse.ArgumentParser(
        description="Visualize a derived LegWheel dataset during rosbag playback."
    )
    parser.add_argument("--dataset", type=Path, required=True)
    options, ros_arguments = parser.parse_known_args(args)
    rclpy.init(args=ros_arguments)
    node = PacketVisualizationNode(options.dataset)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
