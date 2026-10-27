"""Read a raw run and accumulate camera points in the stationary rig frame.

This module has no ROS imports at module scope. Only RosbagSource needs ROS;
configuration, interpolation, cloud construction and seeking can be tested
with ordinary Python objects. Raw recordings are opened read-only.
"""

from bisect import bisect_left
from copy import deepcopy
from dataclasses import dataclass, field
import math
from pathlib import Path

import numpy as np
import yaml

from legwheel_encoder.config import TICKS_TOPIC, EncoderCalibration

from legwheel_dataset.bag_reader import (
    CalibrationTimeline,
    _deserialize,
    _message_types,
    _open_reader,
    message_stamp_ns,
)
from legwheel_dataset.images import decode_image, depth_point_map_in_base
from legwheel_kinematics.imu_pitch import PitchTrajectory, estimate_pitch
from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.encoder_yaw import EncoderYawTrajectory
from legwheel_kinematics.projection import CameraCalibration, project_camera_points
from legwheel_kinematics.transforms import frame_coordinates


TOPIC_CANDIDATES = {
    "depth": ("/legwheel_rgbd/depth/image_raw", "/camera/rgbd/depth/image_raw"),
    "rgb": ("/legwheel_rgbd/color/image_raw", "/camera/rgbd/rgb/image_raw"),
    "depth_info": (
        "/legwheel_rgbd/depth/camera_info",
        "/camera/rgbd/depth/camera_info",
    ),
    "rgb_info": ("/legwheel_rgbd/color/camera_info", "/camera/rgbd/rgb/camera_info"),
    "imu": ("/legwheel_rgbd/gyro_accel/sample", "/camera/imu"),
    "encoder": (TICKS_TOPIC,),
    "wheel": ("/wheel/wheel_state",),
}
POINT_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("rgb", "<u4"),
    ]
)


@dataclass(frozen=True)
class ReplaySettings:
    """Small, explicit controls for cloud detail and association tolerances."""

    pixel_stride: int = 4
    voxel_size_m: float = 0.01
    max_points: int = 500_000
    cloud_rate_hz: float = 5.0
    minimum_depth_m: float = 0.15
    maximum_depth_m: float = 3.0
    rgb_max_delta_ms: float = 50.0
    maximum_pose_gap_s: float = 0.5
    maximum_imu_gap_s: float = 0.05
    maximum_wheel_gap_s: float = 0.05
    maximum_pitch_uncertainty_deg: float = 1.0
    depth_scale_m: float = 0.001

    def __post_init__(self):
        """Reject invalid settings before reading a bag."""
        for name in ("pixel_stride", "max_points"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer.")
        for name in (
            "voxel_size_m",
            "cloud_rate_hz",
            "maximum_depth_m",
            "maximum_pose_gap_s",
            "maximum_imu_gap_s",
            "maximum_wheel_gap_s",
            "maximum_pitch_uncertainty_deg",
            "depth_scale_m",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive.")
        if not 0 <= self.minimum_depth_m < self.maximum_depth_m:
            raise ValueError("Require 0 <= minimum_depth_m < maximum_depth_m.")
        if not math.isfinite(self.rgb_max_delta_ms) or self.rgb_max_delta_ms < 0:
            raise ValueError("rgb_max_delta_ms must be finite and nonnegative.")


@dataclass(frozen=True)
class ImageRecord:
    """Image address in the bag, with acquisition and playback times distinct."""

    topic: str
    log_time_ns: int
    stamp_ns: int
    width: int
    height: int
    frame_id: str


@dataclass(frozen=True)
class CloudFrame:
    """One depth frame, with an optional nearby RGB exposure."""

    depth: ImageRecord
    rgb: ImageRecord | None


class RosbagSource:
    """Index once, then seek directly to individual image records as needed."""

    def __init__(self, directory: Path):
        self.reader = _open_reader(directory)
        self.types = _message_types(self.reader)

    def _select(self, topics, start_ns=0):
        import rosbag2_py

        self.reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
        self.reader.seek(int(start_ns))

    def messages(self, topics):
        """Yield decoded messages without retaining image payloads."""
        self._select(topics)
        while self.reader.has_next():
            topic, data, log_ns = self.reader.read_next()
            yield topic, _deserialize(data, self.types[topic]), int(log_ns)

    def image(self, record: ImageRecord):
        """Read exactly the image selected by topic and bag receipt time."""
        self._select([record.topic], record.log_time_ns)
        if self.reader.has_next():
            topic, data, log_ns = self.reader.read_next()
            if topic == record.topic and int(log_ns) == record.log_time_ns:
                return _deserialize(data, self.types[topic])
        raise ValueError(f"Image not found: {record.topic}@{record.log_time_ns}")

    def close(self):
        """Release the read-only bag handle."""
        self.reader.close()


def _yaml(path):
    with Path(path).open(encoding="utf-8") as stream:
        values = yaml.safe_load(stream)
    if not isinstance(values, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    return values


def _experiment_path(run_directory, experiment_config=None):
    """Resolve geometry and tick calibration from the very same snapshot."""
    if experiment_config is None:
        paths = sorted(Path(run_directory).glob("experiment_config_run_*_snapshot.yaml"))
        if len(paths) != 1:
            raise ValueError(
                "Run must contain one experiment snapshot, or supply --experiment-config."
            )
        experiment_config = paths[0]
    return Path(experiment_config).expanduser()


def load_geometry(run_directory, experiment_config=None):
    """Adapt legacy camera units/names, leaving recorded snapshots untouched."""
    values = deepcopy(_yaml(_experiment_path(run_directory, experiment_config)))
    hardware = values["electronics_hardware"]
    camera = hardware["camera"]["camera_config"]
    notes = []
    if "camera_height_m" not in camera and "sensor_height_mm" in camera:
        camera["camera_height_m"] = camera["sensor_height_mm"] / 1000.0
        notes.append("Legacy sensor_height_mm converted to camera_height_m.")
    if "sensor_front_offset_m" not in camera:
        # Old layouts used a signed X offset (or its positive mm magnitude),
        # with the perpendicular +Z installation offset fixed at 54 mm.
        if "camera_beam_offset_m" in camera:
            camera["sensor_front_offset_m"] = -camera["camera_beam_offset_m"]
            camera["camera_beam_offset_m"] = 0.054
            notes.append(
                "Legacy signed camera_beam_offset_m converted to "
                "sensor_front_offset_m; perpendicular camera_beam_offset_m "
                "uses the legacy 0.054 m installation offset."
            )
        elif "sensor_front_offset_mm" in camera:
            camera["sensor_front_offset_m"] = camera["sensor_front_offset_mm"] / 1000.0
            camera["camera_beam_offset_m"] = 0.054
            notes.append(
                "Legacy sensor_front_offset_mm converted to sensor_front_offset_m; "
                "camera_beam_offset_m uses the legacy 0.054 m installation offset."
            )
    return KinematicsConfig.from_experiment_dict(values), notes


def require_registered_depth(run_directory):
    """Colour projection assumes depth has already been aligned to colour."""
    for pattern in (
        "camera_driver_config_run_*_snapshot.yaml",
        "gemini_336_run_*_snapshot.yaml",
    ):
        paths = sorted(Path(run_directory).glob(pattern))
        if len(paths) == 1:
            config = _yaml(paths[0])
            if (
                config.get("depth_registration") is True
                and str(config.get("align_target_stream", "")).upper() == "COLOR"
            ):
                return
            raise ValueError(
                "3D replay requires depth_registration: true and align_target_stream: COLOR."
            )
    raise ValueError(
        "Cannot find one camera-driver snapshot to verify colour-aligned depth."
    )


def _series(rows, sensor_name):
    """Sort sensor stamps and retain the last value at duplicate timestamps."""
    samples = sorted(dict(rows).items())
    if not samples:
        raise ValueError(f"Recording contains no valid {sensor_name} samples.")
    return np.array([x[0] for x in samples], dtype=np.int64), np.array(
        [x[1] for x in samples]
    )


@dataclass
class RecordedRun:
    """Keep images, calibration, wheel-refined yaw and one IMU pitch estimate."""

    source: object
    geometry: KinematicsConfig
    settings: ReplaySettings
    topics: dict
    frames: list[CloudFrame]
    pitch: PitchTrajectory
    encoder: tuple
    yaw: EncoderYawTrajectory
    depth_calibrations: CalibrationTimeline
    rgb_calibrations: CalibrationTimeline
    notes: list[str] = field(default_factory=list)

    @classmethod
    def load(cls, directory, settings, experiment_config=None, source=None):
        """Read both current and legacy raw runs, without ML packet filtering."""
        directory = Path(directory).expanduser().resolve()
        geometry, notes = load_geometry(directory, experiment_config)
        calibration = EncoderCalibration.from_experiment_dict(
            _yaml(_experiment_path(directory, experiment_config))
        )
        require_registered_depth(directory)
        source = source if source is not None else RosbagSource(directory / "rosbag")
        topics = {}
        for role, candidates in TOPIC_CANDIDATES.items():
            topics[role] = next(
                (name for name in candidates if name in source.types), None
            )
            if topics[role] is None:
                if role == "imu":
                    raise ValueError(
                        "IMU pitch requires a recorded combined accelerometer/gyroscope "
                        f"topic; expected one of {candidates}. "
                        "Record the camera IMU stream for this run."
                    )
                raise ValueError(f"Missing {role} topic; expected one of {candidates}.")
        images = {"depth": [], "rgb": []}
        calibration_rows = {"depth_info": [], "rgb_info": []}
        imu, encoder, wheel = [], [], []
        roles = {topic: role for role, topic in topics.items()}
        for topic, message, log_ns in source.messages(topics.values()):
            role = roles[topic]
            stamp_ns = message_stamp_ns(message, log_ns)
            if role in images:
                images[role].append(
                    ImageRecord(
                        topic,
                        log_ns,
                        stamp_ns,
                        int(message.width),
                        int(message.height),
                        str(message.header.frame_id),
                    )
                )
            elif role in calibration_rows:
                calibration_rows[role].append(
                    (
                        stamp_ns,
                        CameraCalibration.from_message(message),
                        str(message.header.frame_id),
                    )
                )
            elif role == "imu":
                # The Gemini 336 combined stream uses nominal camera optical
                # axes. Reject a declared body frame rather than silently
                # interpreting its measurements with a different rotation.
                frame_id = str(getattr(message.header, "frame_id", ""))
                if frame_id and not frame_id.endswith("optical_frame"):
                    raise ValueError(
                        f"IMU frame {frame_id!r} must use camera optical axes "
                        "(an optical_frame suffix); transform the recorded IMU "
                        "measurements before replaying this run."
                    )
                imu.append(
                    (
                        stamp_ns,
                        (
                            message.angular_velocity.x,
                            message.angular_velocity.y,
                            message.angular_velocity.z,
                            message.linear_acceleration.x,
                            message.linear_acceleration.y,
                            message.linear_acceleration.z,
                        ),
                    )
                )
            elif role == "wheel":
                try:
                    velocity = float(
                        message.velocity[list(message.name).index("wheel_joint")]
                    )
                except (ValueError, IndexError):
                    continue
                # Preserve nonfinite observations as gaps instead of silently
                # integrating through invalid wheel telemetry.
                wheel.append((stamp_ns, velocity))
            elif role == "encoder":
                # Int64 has no header. Its original bag receipt time is the
                # reference used by dataset generation and streamed playback.
                encoder.append((int(log_ns), int(message.data)))
        if not imu:
            raise ValueError("The recorded camera IMU topic contains no measurements.")
        encoder_times, encoder_ticks = _series(encoder, "central-encoder ticks")
        # Counts already include the driver's inversion. Keep the first count
        # in the whole recording as zero, even when viewing a later interval.
        encoder_angles = geometry.theta_y_kinematic_sign * calibration.motion_angle(
            encoder_ticks, encoder_ticks[0]
        )
        encoder_series = encoder_times, encoder_angles
        wheel_times, wheel_velocity = _series(wheel, "wheel odometry")
        # This is the same shared estimator used by the standalone source node.
        # Encoder ticks bound slip drift; integrated measured wheel velocity
        # refines the motion between ticks, across the full run in either direction.
        yaw = EncoderYawTrajectory(
            encoder_times,
            encoder_angles,
            wheel_times,
            wheel_velocity,
            maximum_encoder_gap_s=settings.maximum_pose_gap_s,
            maximum_wheel_gap_s=settings.maximum_wheel_gap_s,
        )
        notes.append(
            "Yaw uses /rotation_encoder/ticks and experiment calibration, with wheel-velocity "
            "integration normalized between encoder changes, as in dataset "
            "generation. Visualization retains the full run and both directions; "
            "intervals without usable wheel refinement keep measured encoder yaw."
        )
        imu_times, imu_values = _series(imu, "camera IMU")
        # Use the full recording, including pauses before/after motion. Running
        # the smoother once also makes random access and backward seeks stable.
        pitch = estimate_pitch(
            imu_times,
            imu_values[:, :3],
            imu_values[:, 3:],
            encoder_times,
            encoder_angles,
            geometry,
            maximum_gap_s=settings.maximum_imu_gap_s,
            maximum_uncertainty_deg=settings.maximum_pitch_uncertainty_deg,
        )
        notes.append(
            "Pitch uses recorded camera IMU fusion and offline smoothing, with "
            "camera_front_angle_deg and encoder circular acceleration compensated. "
            "The IMU is assumed aligned with nominal camera optical axes; the "
            "reported uncertainty is a model estimate, not an accuracy guarantee."
        )
        timelines = {}
        for role, rows in calibration_rows.items():
            timeline = CalibrationTimeline()
            for stamp_ns, calibration, frame_id in sorted(rows, key=lambda x: x[0]):
                timeline.append(stamp_ns, calibration, frame_id)
            if not timeline.calibrations:
                raise ValueError(f"No calibration messages on {topics[role]}.")
            timelines[role] = timeline
        rgb = sorted(images["rgb"], key=lambda x: x.stamp_ns)
        rgb_times = [image.stamp_ns for image in rgb]
        frames = []
        period_ns = int(1e9 / settings.cloud_rate_hz)
        last_log_ns = None
        for depth in images["depth"]:
            if last_log_ns is not None and depth.log_time_ns - last_log_ns < period_ns:
                continue
            right = bisect_left(rgb_times, depth.stamp_ns)
            candidates = rgb[max(0, right - 1) : right + 1]
            colour = (
                min(candidates, key=lambda x: abs(x.stamp_ns - depth.stamp_ns))
                if candidates
                else None
            )
            if (
                colour
                and abs(colour.stamp_ns - depth.stamp_ns)
                > settings.rgb_max_delta_ms * 1e6
            ):
                colour = None
            frames.append(CloudFrame(depth, colour))
            last_log_ns = depth.log_time_ns
        if not frames:
            raise ValueError("Recording contains no depth images.")
        notes.append(
            f"Selected {len(frames)} depth frames; {sum(x.rgb is None for x in frames)} use grey because no RGB exposure is close enough."
        )
        return cls(
            source,
            geometry,
            settings,
            topics,
            frames,
            pitch,
            encoder_series,
            yaw,
            timelines["depth_info"],
            timelines["rgb_info"],
            notes,
        )

    def angles_at(self, stamp_ns):
        """Sample wheel-refined encoder yaw and IMU pitch at acquisition time."""
        yaw = self.yaw.sample(stamp_ns)
        # Keep gyro-supported estimates through impacts or high uncertainty.
        # Only absent estimates (startup or genuine gaps) remain unavailable;
        # those must not be replaced with knee geometry or extrapolated poses.
        return yaw, self.pitch.sample(stamp_ns)

    def frames_at(self, stamp_ns):
        """Return the six measured kinematic poses in the stationary base."""
        yaw, pitch = self.angles_at(stamp_ns)
        return frame_coordinates(self.geometry, pitch, theta_y=yaw)

    def calibration(self, image, *, depth):
        """Check that image geometry matches its calibration epoch."""
        timeline = self.depth_calibrations if depth else self.rgb_calibrations
        calibration = timeline.at(image.stamp_ns)
        if (image.width, image.height) != (calibration.width, calibration.height):
            raise ValueError("Image and CameraInfo dimensions differ.")
        frame_id = timeline.frame_id_at(image.stamp_ns)
        if image.frame_id and frame_id and image.frame_id != frame_id:
            raise ValueError("Image and CameraInfo optical frames differ.")
        return calibration

    def cloud(self, frame):
        """Reconstruct measured depth; never flatten it onto the modelled ground."""
        settings = self.settings
        depth = decode_image(self.source.image(frame.depth))
        depth_m = (
            depth
            if np.issubdtype(depth.dtype, np.floating)
            else depth * settings.depth_scale_m
        )
        mask = np.zeros(depth.shape, dtype=bool)
        mask[:: settings.pixel_stride, :: settings.pixel_stride] = True
        mask &= (depth_m >= settings.minimum_depth_m) & (
            depth_m <= settings.maximum_depth_m
        )
        T_depth = self.frames_at(frame.depth.stamp_ns)[-1]
        point_map = depth_point_map_in_base(
            depth,
            mask,
            self.calibration(frame.depth, depth=True),
            settings.depth_scale_m,
            T_depth,
        )
        points = point_map.points_base_m
        colours = np.full((len(points), 3), 160, dtype=np.uint8)
        if frame.rgb is not None and len(points):
            if frame.rgb.frame_id != frame.depth.frame_id:
                raise ValueError(
                    "Registered RGB and depth must share the colour optical frame."
                )
            rgb_message = self.source.image(frame.rgb)
            rgb = decode_image(rgb_message)
            if rgb.ndim != 3 or rgb.shape[2] < 3:
                raise ValueError("Colour image must have at least three channels.")
            if str(rgb_message.encoding).lower().startswith("bgr"):
                rgb = rgb[:, :, [2, 1, 0]]
            else:
                rgb = rgb[:, :, :3]
            # RGB and aligned depth can have different distortion coefficients.
            # Reproject through RGB calibration instead of copying pixel colours.
            T_rgb = self.frames_at(frame.rgb.stamp_ns)[-1]
            in_rgb = (
                np.linalg.inv(T_rgb) @ np.column_stack((points, np.ones(len(points)))).T
            ).T[:, :3]
            pixels, valid = project_camera_points(
                in_rgb, self.calibration(frame.rgb, depth=False)
            )
            uv = np.floor(np.where(np.isfinite(pixels), pixels, 0)).astype(np.int64)
            colours[valid] = rgb[uv[valid, 1], uv[valid, 0]]
        return points, colours


class VoxelCloud:
    """Keep the first measured point per voxel, with a strict memory bound."""

    def __init__(self, voxel_size_m, max_points):
        self.voxel_size_m = voxel_size_m
        self.max_points = max_points
        self.points = {}
        self.limit_reached = False

    def clear(self):
        """Forget all accumulated geometry when playback moves backward."""
        self.points.clear()
        self.limit_reached = False

    def add(self, points, colours):
        """Add points without averaging away calibration errors or double walls."""
        keys = np.floor(points / self.voxel_size_m).astype(np.int64)
        _, first = np.unique(keys, axis=0, return_index=True)
        for index in sorted(first):
            key = tuple(keys[index])
            if key in self.points:
                continue
            if len(self.points) >= self.max_points:
                self.limit_reached = True
                continue
            r, g, b = map(int, colours[index])
            self.points[key] = (*map(float, points[index]), (r << 16) | (g << 8) | b)

    def packed(self):
        """Return standard little-endian XYZ plus packed RGB, 16 bytes per point."""
        return np.array(list(self.points.values()), dtype=POINT_DTYPE)


class CloudReplay:
    """Advance through bag receipt times, rebuilding history after backward seeks."""

    def __init__(self, run):
        self.run = run
        self.cloud = VoxelCloud(run.settings.voxel_size_m, run.settings.max_points)
        self.next_index = 0
        self.last_time_ns = None
        self.failures = {}

    def advance(self, time_ns, budget=4):
        """Process a bounded batch so the ROS node stays responsive during seeks."""
        changed = False
        if self.last_time_ns is not None and time_ns < self.last_time_ns:
            self.cloud.clear()
            self.next_index = 0
            self.failures.clear()
            changed = True
        self.last_time_ns = time_ns
        for _ in range(budget):
            if self.next_index >= len(self.run.frames):
                break
            frame = self.run.frames[self.next_index]
            if frame.depth.log_time_ns > time_ns:
                break
            try:
                self.cloud.add(*self.run.cloud(frame))
            except ValueError as error:
                self.failures[self.next_index] = str(error)
            self.next_index += 1
            changed = True
        return changed
