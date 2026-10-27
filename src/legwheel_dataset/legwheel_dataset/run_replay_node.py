"""Publish a raw run's accumulated cloud and kinematics for Foxglove 3D."""

import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, TransformStamped
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from legwheel_dataset.run_replay import CloudReplay, RecordedRun, ReplaySettings
from legwheel_kinematics.config import CAMERA_FORWARD_KINEMATIC_DIRECTION
from legwheel_kinematics.projection import undistort_pixels
from legwheel_kinematics.track import generate_track_points
from legwheel_kinematics.transforms import rotation_matrix_to_quaternion


BASE_FRAME = "replay_base"
FRAME_IDS = (
    BASE_FRAME,
    "replay_theta_y_link",
    "replay_theta_p_link",
    "replay_beam_end_link",
    "replay_camera_mount_link",
    "replay_camera_optical_frame",
)
CLOUD_TOPIC = "/legwheel/replay/accumulated_cloud"
SCENE_TOPIC = "/legwheel/replay/scene"
STATUS_TOPIC = "/legwheel/replay/status"


def stamp(timestamp_ns):
    """Build a ROS timestamp from bag time, without using the host clock."""
    return Time(
        sec=int(timestamp_ns // 1_000_000_000),
        nanosec=int(timestamp_ns % 1_000_000_000),
    )


def point(xyz):
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


def colour(r, g, b):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)


def cloud_message(packed, timestamp_ns):
    """Publish explicit XYZ float32 / RGB uint32 fields; no PCL dependency."""
    message = PointCloud2()
    message.header.frame_id = BASE_FRAME
    message.header.stamp = stamp(timestamp_ns)
    message.height = 1
    message.width = len(packed)
    message.fields = [
        PointField(name=name, offset=offset, datatype=datatype, count=1)
        for name, offset, datatype in (
            ("x", 0, PointField.FLOAT32),
            ("y", 4, PointField.FLOAT32),
            ("z", 8, PointField.FLOAT32),
            ("rgb", 12, PointField.UINT32),
        )
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = 16 * len(packed)
    message.data = packed.tobytes()
    message.is_dense = True
    return message


class RunReplayNode(Node):
    """Follow external rosbag /clock; all geometry uses a separate frame tree."""

    def __init__(self, run, alpha_off_rad, alpha_sp_rad):
        super().__init__("legwheel_run_replay")
        self.run = run
        self.replay = CloudReplay(run)
        self.alpha_off_rad = alpha_off_rad
        self.alpha_sp_rad = alpha_sp_rad
        self.track = generate_track_points(run.geometry)
        self.clock_ns = None
        self.last_cloud_wall = -float("inf")
        self.reported_failures = set()
        self.reported_limit = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_publisher = self.create_publisher(PointCloud2, CLOUD_TOPIC, latched)
        self.scene_publisher = self.create_publisher(MarkerArray, SCENE_TOPIC, latched)
        self.status_publisher = self.create_publisher(String, STATUS_TOPIC, latched)
        self.transforms = TransformBroadcaster(self)
        self.create_subscription(
            Clock, "/clock", self.on_clock, qos_profile_sensor_data
        )
        # A steady timer also runs while bag time is paused. It finishes seek
        # rebuilds and lets clients connect after the last camera frame.
        self.create_timer(
            0.1, self.update, clock=RosClock(clock_type=ClockType.STEADY_TIME)
        )
        for note in run.notes:
            self.get_logger().info(note)
        self.get_logger().info(
            f"READY: indexed {len(run.frames)} cloud frames. Start ros2 bag play with --clock. "
            f"Foxglove fixed/display frame: {BASE_FRAME}."
        )

    def on_clock(self, message):
        self.clock_ns = int(message.clock.sec) * 1_000_000_000 + int(
            message.clock.nanosec
        )

    def update(self):
        if self.clock_ns is None:
            return
        previous = self.replay.last_time_ns
        if previous is not None and self.clock_ns < previous:
            self.reported_failures.clear()
            self.reported_limit = False
        changed = self.replay.advance(self.clock_ns)
        for index, reason in self.replay.failures.items():
            if index not in self.reported_failures:
                self.get_logger().warning(f"Skipped depth frame {index}: {reason}")
                self.reported_failures.add(index)
        if self.replay.cloud.limit_reached and not self.reported_limit:
            self.get_logger().warning(
                "Point limit reached; increase --voxel-size-m or --max-points to retain more terrain."
            )
            self.reported_limit = True
        if changed or time.monotonic() - self.last_cloud_wall >= 1.0:
            self.cloud_publisher.publish(
                cloud_message(self.replay.cloud.packed(), self.clock_ns)
            )
            self.last_cloud_wall = time.monotonic()
        self.scene_publisher.publish(self.scene())
        self.status_publisher.publish(
            String(
                data=(
                    f"Frames {self.replay.next_index}/{len(self.run.frames)}; "
                    f"voxels {len(self.replay.cloud.points)}; "
                    f"skipped {len(self.replay.failures)}; "
                    f"limit reached: {self.replay.cloud.limit_reached}"
                )
            )
        )

    def marker(self, marker_id, name, marker_type, rgb):
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = stamp(self.clock_ns)
        marker.ns = name
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color = colour(*rgb)
        marker.scale.x = 0.005
        return marker

    def line(self, marker_id, name, points, rgb, *, closed=False):
        marker = self.marker(marker_id, name, Marker.LINE_STRIP, rgb)
        marker.points = [point(xyz) for xyz in points]
        if closed and marker.points:
            marker.points.append(marker.points[0])
        return marker

    def scene(self):
        """Draw explicit axes and labels as well as publishing the TF tree."""
        clear = self.marker(0, "clear", Marker.LINE_STRIP, (1, 1, 1))
        clear.action = Marker.DELETEALL
        markers = [clear]
        for index, name, points, rgb in (
            (1, "inner_track", self.track.inner_track_points, (0.1, 0.8, 1.0)),
            (2, "outer_track", self.track.outer_track_points, (1.0, 0.5, 0.1)),
        ):
            markers.append(self.line(index, name, points, rgb, closed=True))
        try:
            frames = self.run.frames_at(self.clock_ns)
            yaw, _ = self.run.angles_at(self.clock_ns)
        except ValueError:
            return MarkerArray(markers=markers)
        origins = np.array([frame[:3, 3] for frame in frames])
        markers.append(self.line(3, "kinematic_chain", origins, (0.9, 0.9, 0.1)))
        for axis, rgb in enumerate(((1, 0, 0), (0, 1, 0), (0, 0.4, 1))):
            axes = self.marker(4 + axis, f"axis_{axis}", Marker.LINE_LIST, rgb)
            for frame in frames:
                origin = frame[:3, 3]
                axes.points.extend(
                    [point(origin), point(origin + 0.10 * frame[:3, axis])]
                )
            markers.append(axes)
        for index, (name, origin) in enumerate(zip(FRAME_IDS, origins)):
            label = self.marker(
                10 + index, "frame_labels", Marker.TEXT_VIEW_FACING, (1, 1, 1)
            )
            # F1/F2 and mount/optical frames share origins. Alternate label
            # heights so both names remain visible at those locations.
            label.pose.position = point(
                origin + np.array([0, 0, 0.025 + (index % 2) * 0.03])
            )
            label.scale.z = 0.025
            label.text = name
            markers.append(label)
        centre = yaw + CAMERA_FORWARD_KINEMATIC_DIRECTION * self.alpha_off_rad
        angles = np.linspace(
            centre - self.alpha_sp_rad / 2, centre + self.alpha_sp_rad / 2, 24
        )
        patch_edges = [
            np.column_stack(
                (
                    radius * np.cos(angles),
                    radius * np.sin(angles),
                    np.zeros(len(angles)),
                )
            )
            for radius in (
                self.run.geometry.track_inner_radius_m,
                self.run.geometry.track_outer_radius_m,
            )
        ]
        markers.append(
            self.line(
                20,
                "preview_patch",
                np.vstack((patch_edges[0], patch_edges[1][::-1])),
                (1, 0.1, 0.1),
                closed=True,
            )
        )
        calibration = self.run.rgb_calibrations.at(self.clock_ns)
        corners = np.array(
            [
                [0, 0],
                [calibration.width - 1, 0],
                [calibration.width - 1, calibration.height - 1],
                [0, calibration.height - 1],
            ]
        )
        rays = (
            np.column_stack((undistort_pixels(corners, calibration), np.ones(4))) * 0.20
        )
        T_camera = frames[-1]
        corners_base = (T_camera @ np.column_stack((rays, np.ones(4))).T).T[:, :3]
        frustum = self.marker(21, "camera_frustum", Marker.LINE_LIST, (0.7, 0.3, 1))
        for index in range(4):
            frustum.points.extend(
                [
                    point(T_camera[:3, 3]),
                    point(corners_base[index]),
                    point(corners_base[index]),
                    point(corners_base[(index + 1) % 4]),
                ]
            )
        markers.append(frustum)
        transforms = []
        for index in range(5):
            relative = np.linalg.inv(frames[index]) @ frames[index + 1]
            transform = TransformStamped()
            transform.header.frame_id = FRAME_IDS[index]
            transform.child_frame_id = FRAME_IDS[index + 1]
            transform.header.stamp = stamp(self.clock_ns)
            x, y, z = map(float, relative[:3, 3])
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            x, y, z, w = rotation_matrix_to_quaternion(relative[:3, :3])
            transform.transform.rotation.x = x
            transform.transform.rotation.y = y
            transform.transform.rotation.z = z
            transform.transform.rotation.w = w
            transforms.append(transform)
        self.transforms.sendTransform(transforms)
        return MarkerArray(markers=markers)


def main(args=None):
    """Start the viewer first, wait for READY, then play the raw bag with /clock."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--run-directory", type=Path, required=True)
    parser.add_argument(
        "--experiment-config",
        type=Path,
        help="Optional geometry override; never modifies the recording.",
    )
    defaults = ReplaySettings()
    for name in defaults.__dataclass_fields__:
        default = getattr(defaults, name)
        parser.add_argument(
            "--" + name.replace("_", "-"),
            type=type(default),
            default=default,
            help=name.replace("_", " "),
        )
    parser.add_argument("--alpha-off-rad", type=float, default=0.35)
    parser.add_argument("--alpha-sp-rad", type=float, default=0.10)
    options, ros_args = parser.parse_known_args(args)
    if not (
        math.isfinite(options.alpha_off_rad)
        and math.isfinite(options.alpha_sp_rad)
        and options.alpha_off_rad >= 0
        and 0 < options.alpha_sp_rad < 2 * math.pi
    ):
        parser.error("Require finite alpha_off_rad >= 0 and 0 < alpha_sp_rad < 2*pi.")
    try:
        settings = ReplaySettings(
            **{name: getattr(options, name) for name in defaults.__dataclass_fields__}
        )
        print(
            "Indexing run metadata (images are not retained in memory)...", flush=True
        )
        run = RecordedRun.load(
            options.run_directory, settings, options.experiment_config
        )
    except (ValueError, KeyError, OSError, RuntimeError) as error:
        parser.error(str(error))
    rclpy.init(args=[sys.argv[0], *ros_args])
    node = None
    try:
        node = RunReplayNode(run, options.alpha_off_rad, options.alpha_sp_rad)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        run.source.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
