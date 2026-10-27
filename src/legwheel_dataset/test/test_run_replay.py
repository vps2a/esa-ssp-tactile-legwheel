"""Exercise raw-run compatibility, actual cloud geometry and seek semantics."""

from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from legwheel_dataset.bag_reader import message_stamp_ns
from legwheel_dataset.run_replay import (
    CloudFrame,
    CloudReplay,
    ImageRecord,
    POINT_DTYPE,
    RecordedRun,
    ReplaySettings,
    RosbagSource,
    TOPIC_CANDIDATES,
    VoxelCloud,
    load_geometry,
)
from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.transforms import frame_coordinates


def header(stamp_ns):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9),
        frame_id="camera_color_optical_frame",
    )


def image(array, stamp_ns, encoding):
    return SimpleNamespace(
        header=header(stamp_ns),
        width=array.shape[1],
        height=array.shape[0],
        encoding=encoding,
        is_bigendian=False,
        step=array.strides[0],
        data=array.tobytes(),
    )


def camera_info(stamp_ns, width=4, height=4, focal=10.0):
    return SimpleNamespace(
        header=header(stamp_ns),
        width=width,
        height=height,
        k=[focal, 0, 0, 0, focal, 0, 0, 0, 1],
        d=[0.0] * 5,
        distortion_model="plumb_bob",
    )


def experiment():
    return {
        "rig_config": {"pivot_height_m": 0.353, "beam_radius_m": 0.83},
        "leg_config": {"hip_link_length_m": 0.2, "calf_link_length_m": 0.2},
        "wheel_config": {"wheel_radius_mm": 90.0, "wheel_width_mm": 100.0},
        "electronics_hardware": {
            "encoder_setup": {
                "theta_y_kinematic_sign": -1,
                # One tick per 0.01 rad makes the synthetic 100 Hz turn exact.
                "ticks_per_legwheel_revolution": 200.0 * math.pi,
            },
            "camera": {
                "camera_config": {
                    "camera_height_m": 0.029,
                    "sensor_front_offset_m": 0.043,
                    "camera_beam_offset_m": 0.054,
                    "camera_front_angle_deg": 40.0,
                }
            },
        },
    }


class FakeSource:
    def __init__(
        self, legacy=False, *, pitch=0.12, camera_angle_deg=40.0, encoder_sign=-1
    ):
        self.topics = {
            role: names[-1] if legacy else names[0]
            for role, names in TOPIC_CANDIDATES.items()
        }
        self.types = dict.fromkeys(self.topics.values(), "unused")
        self.rows = []
        self.image_reads = []
        geometry_values = experiment()
        geometry_values["electronics_hardware"]["encoder_setup"][
            "theta_y_kinematic_sign"
        ] = encoder_sign
        geometry_values["electronics_hardware"]["camera"]["camera_config"][
            "camera_front_angle_deg"
        ] = camera_angle_deg
        geometry = KinematicsConfig.from_experiment_dict(geometry_values)
        # Simulate real camera specific force during a constant-speed turn.
        # Nonzero gravity and centripetal acceleration make these fixtures check
        # the sensor convention, encoder sign and camera mounting compensation.
        for index in range(201):
            elapsed_s = index / 100
            stamp_ns = 10**10 + index * 10**7
            self.rows.append(
                (
                    self.topics["wheel"],
                    SimpleNamespace(
                        header=header(stamp_ns),
                        name=["wheel_joint"],
                        velocity=[-2.0],
                    ),
                    stamp_ns + 1_500_000,
                )
            )
            self.rows.append(
                (
                    self.topics["encoder"],
                    SimpleNamespace(
                        data=120 + index,
                    ),
                    stamp_ns,
                )
            )
            camera_pose = frame_coordinates(
                geometry, pitch, theta_y=encoder_sign * elapsed_s
            )[-1]
            rotation = camera_pose[:3, :3]
            position = camera_pose[:3, 3]
            acceleration = np.array([-position[0], -position[1], 0.0])
            specific_force = rotation.T @ (acceleration + [0.0, 0.0, 9.80665])
            angular_velocity = rotation.T @ [0.0, 0.0, encoder_sign]
            self.rows.append(
                (
                    self.topics["imu"],
                    SimpleNamespace(
                        header=header(stamp_ns),
                        angular_velocity=SimpleNamespace(
                            **dict(zip(("x", "y", "z"), angular_velocity))
                        ),
                        linear_acceleration=SimpleNamespace(
                            **dict(zip(("x", "y", "z"), specific_force))
                        ),
                    ),
                    stamp_ns + 2_000_000,
                )
            )
        self.rows.extend(
            [
                (self.topics["depth_info"], camera_info(10**10), 10**10),
                (self.topics["rgb_info"], camera_info(10**10), 10**10),
            ]
        )
        # Use interior pixels so tiny round-off from pose inversion cannot put
        # a theoretically on-border projection just outside the image bounds.
        depth = np.zeros((4, 4), dtype=np.uint16)
        depth[1, 1], depth[3, 1], depth[2, 2] = 1000, 2000, 4000
        rgb = np.zeros((4, 4, 3), dtype=np.uint8)
        rgb[:2], rgb[2:] = [255, 0, 0], [0, 0, 255]
        for stamp_ns in (10_500_000_000, 11_500_000_000):
            self.rows.append(
                (
                    self.topics["depth"],
                    image(depth, stamp_ns, "16UC1"),
                    stamp_ns + 50_000_000,
                )
            )
        self.rows.append(
            (self.topics["rgb"], image(rgb, 10_500_000_000, "rgb8"), 10_510_000_000)
        )
        self.rows.sort(key=lambda row: row[2])

    def messages(self, topics):
        return (row for row in self.rows if row[0] in topics)

    def image(self, record):
        self.image_reads.append(record)
        return next(
            message
            for topic, message, log_ns in self.rows
            if topic == record.topic and log_ns == record.log_time_ns
        )


class RecordedRunTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.config_path = self.directory / "experiment_config_run_1_snapshot.yaml"
        self.config_path.write_text(yaml.safe_dump(experiment()))
        (self.directory / "camera_driver_config_run_1_snapshot.yaml").write_text(
            "depth_registration: true\nalign_target_stream: COLOR\n"
        )

    def load(self, source=None, **settings):
        return RecordedRun.load(
            self.directory,
            ReplaySettings(pixel_stride=1, **settings),
            source=source if source is not None else FakeSource(),
        )

    def test_legacy_geometry_and_topics_match_current_without_changing_snapshot(self):
        current, _ = load_geometry(self.directory)
        legacy = deepcopy(experiment())
        camera = legacy["electronics_hardware"]["camera"]["camera_config"]
        del (
            camera["camera_height_m"],
            camera["camera_beam_offset_m"],
            camera["sensor_front_offset_m"],
        )
        camera.update(sensor_height_mm=29, sensor_front_offset_mm=43)
        original = yaml.safe_dump(legacy)
        self.config_path.write_text(original)
        run = self.load(FakeSource(legacy=True))
        self.assertEqual(run.geometry, current)
        self.assertEqual(self.config_path.read_text(), original)
        self.assertEqual(run.topics["depth"], "/camera/rgbd/depth/image_raw")
        self.assertTrue(any("0.054 m" in note for note in run.notes))

    def test_old_signed_x_offset_migrates_without_changing_snapshot(self):
        current, _ = load_geometry(self.directory)
        legacy = experiment()
        camera = legacy["electronics_hardware"]["camera"]["camera_config"]
        del camera["sensor_front_offset_m"]
        camera["camera_beam_offset_m"] = -0.043
        original = yaml.safe_dump(legacy)
        self.config_path.write_text(original)
        geometry, notes = load_geometry(self.directory)
        self.assertEqual(geometry, current)
        self.assertEqual(self.config_path.read_text(), original)
        self.assertTrue(any("Legacy signed" in note for note in notes))

    def test_explicit_offsets_override_legacy_values_and_defaults(self):
        values = experiment()
        camera = values["electronics_hardware"]["camera"]["camera_config"]
        camera.update(
            sensor_front_offset_m=0.031,
            camera_beam_offset_m=0.067,
            sensor_front_offset_mm=999,
        )
        override = self.directory / "corrected.yaml"
        original = self.config_path.read_text()
        override.write_text(yaml.safe_dump(values))
        geometry, notes = load_geometry(self.directory, override)
        self.assertEqual(geometry.sensor_front_offset, 0.031)
        self.assertEqual(geometry.camera_beam_offset, 0.067)
        self.assertEqual(notes, [])
        self.assertEqual(self.config_path.read_text(), original)

    def test_camera_offsets_must_be_finite_numbers(self):
        for field in ("sensor_front_offset_m", "camera_beam_offset_m"):
            for invalid in (None, True, "0.043", float("nan"), float("inf")):
                with self.subTest(field=field, invalid=invalid):
                    values = experiment()
                    values["electronics_hardware"]["camera"]["camera_config"][field] = (
                        invalid
                    )
                    self.config_path.write_text(yaml.safe_dump(values))
                    with self.assertRaisesRegex(ValueError, field):
                        load_geometry(self.directory)

    def test_index_retains_no_images_and_geometry_does_not_require_rgb_pair(self):
        source = FakeSource()
        run = self.load(source)
        self.assertEqual(source.image_reads, [])
        self.assertEqual(len(run.frames), 2)
        self.assertIsNotNone(run.frames[0].rgb)
        self.assertIsNone(run.frames[1].rgb)
        points, colours = run.cloud(run.frames[1])
        self.assertEqual(points.shape, (2, 3))  # zero and 4 m pixels rejected
        np.testing.assert_array_equal(colours, np.full((2, 3), 160))

    def test_point_geometry_and_colours_use_the_image_acquisition_pose(self):
        run = self.load()
        frame = run.frames[0]
        points, colours = run.cloud(frame)
        T_camera = run.frames_at(frame.depth.stamp_ns)[-1]
        expected_camera = np.array([[0.1, 0.1, 1, 1], [0.2, 0.6, 2, 1]])
        np.testing.assert_allclose(points, (T_camera @ expected_camera.T).T[:, :3])
        np.testing.assert_array_equal(colours, [[255, 0, 0], [0, 0, 255]])
        yaw, _ = run.angles_at(frame.depth.stamp_ns)
        self.assertAlmostEqual(yaw, -0.5)

    def test_pitch_uses_imu_without_leg_topic_and_compensates_camera_mount_angle(self):
        for pitch, camera_angle, encoder_sign in (
            (0.12, 40.0, -1),
            (-0.10, -25.0, -1),
            (0.0, 65.0, 1),
        ):
            with self.subTest(pitch=pitch, camera_angle=camera_angle):
                values = experiment()
                values["electronics_hardware"]["encoder_setup"][
                    "theta_y_kinematic_sign"
                ] = encoder_sign
                values["electronics_hardware"]["camera"]["camera_config"][
                    "camera_front_angle_deg"
                ] = camera_angle
                self.config_path.write_text(yaml.safe_dump(values))
                run = self.load(
                    FakeSource(
                        pitch=pitch,
                        camera_angle_deg=camera_angle,
                        encoder_sign=encoder_sign,
                    )
                )
                self.assertNotIn("leg", run.topics)
                for frame in run.frames:
                    self.assertAlmostEqual(
                        run.angles_at(frame.depth.stamp_ns)[1],
                        pitch,
                        delta=math.radians(0.1),
                    )
                    self.assertAlmostEqual(
                        run.angles_at(frame.depth.stamp_ns)[0],
                        encoder_sign * (frame.depth.stamp_ns / 1e9 - 10.0),
                    )
                self.assertTrue(
                    any("camera_front_angle_deg" in note for note in run.notes)
                )

    def test_missing_imu_requires_a_recorded_camera_stream(self):
        source = FakeSource()
        del source.types[source.topics["imu"]]
        with self.assertRaisesRegex(ValueError, "Record the camera IMU"):
            self.load(source)

    def test_missing_wheel_topic_requires_recorded_odometry(self):
        source = FakeSource()
        del source.types[source.topics["wheel"]]
        with self.assertRaisesRegex(ValueError, "Missing wheel topic"):
            self.load(source)

    def test_joint_state_angles_cannot_replace_missing_recorded_ticks(self):
        source = FakeSource()
        del source.types[source.topics["encoder"]]
        source.types["/rotation_encoder/joint_state"] = "sensor_msgs/msg/JointState"
        with self.assertRaisesRegex(ValueError, "Missing encoder topic"):
            self.load(source)

    def test_reconstruction_uses_tick_calibration_and_original_bag_timestamps(self):
        source = FakeSource()
        # A misleading header must not replace a tick's recorded receipt time.
        for topic, message, _ in source.rows:
            if topic == source.topics["encoder"]:
                message.header = header(1)
        values = experiment()
        values["electronics_hardware"]["encoder_setup"]["ticks_per_legwheel_revolution"] *= 2
        self.config_path.write_text(yaml.safe_dump(values))
        run = self.load(source)
        self.assertEqual(int(run.encoder[0][0]), 10**10)
        self.assertAlmostEqual(run.angles_at(10_500_000_000)[0], -0.25)

    def test_yaw_uses_wheel_refinement_between_recorded_tick_changes(self):
        source = FakeSource()
        for topic, message, log_ns in source.rows:
            elapsed = (message_stamp_ns(message, log_ns) - 10**10) * 1e-9
            if topic == source.topics["encoder"]:
                message.data = 400 + 100 * math.floor(elapsed)
            elif topic == source.topics["wheel"]:
                message.velocity = [-2.0 * (1.0 + elapsed)]
        run = self.load(source)
        # In the first second, integral(0, .5) / integral(0, 1) = .625/1.5.
        # The configured sign is -1 and the initial encoder count is the origin.
        expected = -0.625 / 1.5
        self.assertAlmostEqual(run.angles_at(10_500_000_000)[0], expected)
        self.assertAlmostEqual(run.angles_at(11_000_000_000)[0], -1.0)
        np.testing.assert_allclose(
            run.frames_at(10_500_000_000),
            frame_coordinates(run.geometry, run.pitch.sample(10_500_000_000), expected),
        )

    def test_wheel_gap_keeps_encoder_poses_and_does_not_reject_clouds(self):
        source = FakeSource()
        for topic, message, log_ns in source.rows:
            elapsed = (message_stamp_ns(message, log_ns) - 10**10) * 1e-9
            if topic == source.topics["encoder"]:
                message.data = 100 * math.floor(elapsed)
        source.rows = [
            row
            for row in source.rows
            if row[0] != source.topics["wheel"]
            or not 10_400_000_000 < message_stamp_ns(row[1], row[2]) < 10_600_000_000
        ]
        run = self.load(source)
        self.assertEqual(run.angles_at(10_500_000_000)[0], 0.0)
        # Wheel refinement resumes on the following fully supported interval.
        self.assertAlmostEqual(run.angles_at(11_500_000_000)[0], -1.5)
        replay = CloudReplay(run)
        replay.advance(run.frames[-1].depth.log_time_ns)
        self.assertEqual(replay.failures, {})
        self.assertGreater(len(replay.cloud.points), 0)

    def test_low_confidence_pitch_does_not_freeze_frames_or_reject_clouds(self):
        run = self.load()
        run.pitch.valid[:] = False
        run.pitch.uncertainty_rad[:] = math.radians(8.0)
        for frame in run.frames:
            yaw, pitch = run.angles_at(frame.depth.stamp_ns)
            self.assertTrue(math.isfinite(yaw) and math.isfinite(pitch))
            self.assertTrue(np.isfinite(run.frames_at(frame.depth.stamp_ns)).all())
        replay = CloudReplay(run)
        replay.advance(run.frames[-1].depth.log_time_ns)
        self.assertEqual(replay.failures, {})
        self.assertGreater(len(replay.cloud.points), 0)

    def test_imu_declared_in_a_body_frame_is_rejected(self):
        source = FakeSource()
        for topic, message, _ in source.rows:
            if topic == source.topics["imu"]:
                message.header.frame_id = "camera_imu_frame"
                break
        with self.assertRaisesRegex(ValueError, "camera optical axes"):
            self.load(source)

    def test_imu_gap_rejects_frame_instead_of_falling_back_to_joint_geometry(self):
        source = FakeSource()
        source.rows = [
            row
            for row in source.rows
            if row[0] != source.topics["imu"]
            or not 10_440_000_000 < message_stamp_ns(row[1], row[2]) < 10_560_000_000
        ]
        run = self.load(source)
        with self.assertRaisesRegex(ValueError, "gap|coverage|outside"):
            run.angles_at(run.frames[0].depth.stamp_ns)
        replay = CloudReplay(run)
        replay.advance(run.frames[-1].depth.log_time_ns)
        self.assertIn(0, replay.failures)
        self.assertNotIn(1, replay.failures)

    def test_rgb_is_reprojected_instead_of_assuming_equal_intrinsics(self):
        source = FakeSource()
        for index, (topic, message, log_ns) in enumerate(source.rows):
            if topic == source.topics["rgb_info"]:
                source.rows[index] = (topic, camera_info(10**10, 8, 8, 20.0), log_ns)
            elif topic == source.topics["rgb"]:
                rgb = np.zeros((8, 8, 3), dtype=np.uint8)
                rgb[:4] = [255, 0, 0]
                rgb[4:] = [0, 255, 0]
                source.rows[index] = (topic, image(rgb, 10_500_000_000, "rgb8"), log_ns)
        run = self.load(source)
        _, colours = run.cloud(run.frames[0])
        np.testing.assert_array_equal(colours, [[255, 0, 0], [0, 255, 0]])

    def test_bgr_images_are_converted_to_rgb(self):
        source = FakeSource()
        for index, (topic, message, log_ns) in enumerate(source.rows):
            if topic == source.topics["rgb"]:
                array = np.full((4, 4, 3), [10, 20, 30], dtype=np.uint8)
                source.rows[index] = (
                    topic,
                    image(array, 10_500_000_000, "bgr8"),
                    log_ns,
                )
        run = self.load(source)
        _, colours = run.cloud(run.frames[0])
        np.testing.assert_array_equal(colours, [[30, 20, 10], [30, 20, 10]])

    def test_unregistered_depth_is_rejected(self):
        (self.directory / "camera_driver_config_run_1_snapshot.yaml").write_text(
            "depth_registration: false\nalign_target_stream: COLOR\n"
        )
        with self.assertRaisesRegex(ValueError, "depth_registration"):
            self.load()

    def test_calibration_mismatch_is_rejected(self):
        run = self.load()
        run.depth_calibrations.calibrations[0].K[0, 0] = 20
        run.depth_calibrations.frame_ids[0] = "wrong_optical_frame"
        with self.assertRaisesRegex(ValueError, "optical frames"):
            run.cloud(run.frames[0])


class AccumulationTest(unittest.TestCase):
    def test_voxels_preserve_first_measurement_negative_coordinates_and_limit(self):
        cloud = VoxelCloud(0.1, 2)
        cloud.add(
            np.array([[0.01, 0, 1], [0.02, 0, 1], [-0.01, 0, 1], [2, 0, 1]]),
            np.array([[10, 20, 30], [255, 0, 0], [0, 255, 0], [0, 0, 255]]),
        )
        packed = cloud.packed()
        self.assertEqual(packed.dtype, POINT_DTYPE)
        self.assertEqual(packed.dtype.itemsize, 16)
        self.assertEqual(len(packed), 2)
        self.assertTrue(cloud.limit_reached)
        x, y, z, rgb = struct.unpack("<fffI", packed[:1].tobytes())
        self.assertAlmostEqual(x, 0.01)
        self.assertEqual((y, z, rgb), (0.0, 1.0, 0x0A141E))
        cloud.clear()
        self.assertEqual(cloud.packed().tobytes(), b"")
        self.assertFalse(cloud.limit_reached)

    def test_forward_backward_pause_and_loop_never_keep_future_points(self):
        frames = [
            CloudFrame(ImageRecord("depth", t, t - 20, 1, 1, "camera"), None)
            for t in (100, 200, 300)
        ]
        reads = []

        def load(frame):
            reads.append(frame.depth.stamp_ns)
            return np.array([[frame.depth.log_time_ns, 0, 0]]), np.array([[1, 2, 3]])

        run = SimpleNamespace(frames=frames, settings=ReplaySettings(), cloud=load)
        replay = CloudReplay(run)
        replay.advance(80)
        self.assertEqual(reads, [])  # schedule by receipt time, not sensor stamp
        replay.advance(300, budget=1)
        self.assertEqual(replay.next_index, 1)
        replay.advance(300)
        complete = replay.cloud.packed().tobytes()
        replay.advance(300)
        self.assertEqual(len(reads), 3)  # pause does not add duplicates
        replay.advance(150)
        self.assertEqual(replay.next_index, 1)
        self.assertEqual(len(replay.cloud.points), 1)
        replay.advance(50)
        self.assertEqual(replay.next_index, 0)
        self.assertEqual(len(replay.cloud.points), 0)
        replay.advance(300)
        self.assertEqual(replay.cloud.packed().tobytes(), complete)

    def test_bad_frame_is_reported_and_does_not_stop_remaining_history(self):
        frames = [
            CloudFrame(ImageRecord("depth", t, t, 1, 1, "camera"), None)
            for t in (100, 200)
        ]

        def load(frame):
            if frame.depth.log_time_ns == 100:
                raise ValueError("bad calibration")
            return np.array([[1, 2, 3]]), np.array([[1, 2, 3]])

        replay = CloudReplay(
            SimpleNamespace(frames=frames, settings=ReplaySettings(), cloud=load)
        )
        replay.advance(300)
        self.assertEqual(replay.failures, {0: "bad calibration"})
        self.assertEqual(len(replay.cloud.points), 1)

    def test_settings_reject_invalid_limits(self):
        for kwargs in (
            {"pixel_stride": 0},
            {"max_points": -1},
            {"voxel_size_m": 0},
            {"cloud_rate_hz": float("nan")},
            {"minimum_depth_m": 4},
            {"rgb_max_delta_ms": -1},
            {"maximum_imu_gap_s": 0},
            {"maximum_pitch_uncertainty_deg": float("nan")},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                ReplaySettings(**kwargs)


class RosbagSourceTest(unittest.TestCase):
    def test_seek_addresses_images_by_topic_and_receipt_time(self):
        rows = [
            ("depth", "depth_payload", 100),
            ("rgb", "rgb_payload", 120),
        ]

        class Reader:
            def get_all_topics_and_types(self):
                return [
                    SimpleNamespace(name=name, type="Image")
                    for name in ("depth", "rgb")
                ]

            def set_filter(self, selected):
                self.topics = selected.topics

            def seek(self, timestamp):
                self.pending = [
                    row for row in rows if row[0] in self.topics and row[2] >= timestamp
                ]

            def has_next(self):
                return bool(self.pending)

            def read_next(self):
                return self.pending.pop(0)

            def close(self):
                self.closed = True

        reader = Reader()
        with (
            patch.dict(
                sys.modules,
                {
                    "rosbag2_py": SimpleNamespace(
                        StorageFilter=lambda topics: SimpleNamespace(topics=topics)
                    )
                },
            ),
            patch("legwheel_dataset.run_replay._open_reader", return_value=reader),
            patch(
                "legwheel_dataset.run_replay._deserialize",
                side_effect=lambda data, _: data,
            ),
        ):
            source = RosbagSource(Path("unused"))
            self.assertEqual(list(source.messages(["depth"])), [rows[0]])
            self.assertEqual(
                source.image(ImageRecord("rgb", 120, 80, 1, 1, "camera")), "rgb_payload"
            )
            self.assertEqual(
                source.image(ImageRecord("depth", 100, 50, 1, 1, "camera")),
                "depth_payload",
            )
            with self.assertRaisesRegex(ValueError, "not found"):
                source.image(ImageRecord("depth", 101, 51, 1, 1, "camera"))
            source.close()
            self.assertTrue(reader.closed)


if __name__ == "__main__":
    unittest.main()
