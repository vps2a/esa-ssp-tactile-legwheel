"""Exercise raw-run compatibility, actual cloud geometry and seek semantics."""

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

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
    sample_angle,
)


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


def camera_info(stamp_ns, width=2, height=2, focal=10.0):
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
            "encoder_setup": {"theta_y_kinematic_sign": -1},
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
    def __init__(self, legacy=False):
        self.topics = {
            role: names[-1] if legacy else names[0]
            for role, names in TOPIC_CANDIDATES.items()
        }
        self.types = dict.fromkeys(self.topics.values(), "unused")
        self.rows = []
        self.image_reads = []
        for index in range(21):
            stamp_ns = 10**10 + index * 10**8
            for role, name, angle in (
                ("leg", "knee_joint", 0.8 + index / 100),
                ("encoder", "rotation_joint", index / 10),
            ):
                self.rows.append(
                    (
                        self.topics[role],
                        SimpleNamespace(
                            header=header(stamp_ns),
                            name=[name],
                            position=[angle],
                        ),
                        stamp_ns + 1_000_000,
                    )
                )
        self.rows.extend(
            [
                (self.topics["depth_info"], camera_info(10**10), 10**10),
                (self.topics["rgb_info"], camera_info(10**10), 10**10),
            ]
        )
        depth = np.array([[1000, 0], [2000, 4000]], dtype=np.uint16)
        rgb = np.array(
            [[[255, 0, 0], [0, 255, 0]], [[0, 0, 255], [255, 255, 0]]], dtype=np.uint8
        )
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
        del legacy["electronics_hardware"]["encoder_setup"]["theta_y_kinematic_sign"]
        original = yaml.safe_dump(legacy)
        self.config_path.write_text(original)
        run = self.load(FakeSource(legacy=True))
        self.assertEqual(run.geometry, current)
        self.assertEqual(self.config_path.read_text(), original)
        self.assertEqual(run.topics["depth"], "/camera/rgbd/depth/image_raw")
        self.assertTrue(any("sign defaults" in note for note in run.notes))
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
        expected_camera = np.array([[0, 0, 1, 1], [0, 0.2, 2, 1]])
        np.testing.assert_allclose(points, (T_camera @ expected_camera.T).T[:, :3])
        np.testing.assert_array_equal(colours, [[255, 0, 0], [0, 0, 255]])
        yaw, _ = run.angles_at(frame.depth.stamp_ns)
        self.assertAlmostEqual(yaw, -0.5)

    def test_rgb_is_reprojected_instead_of_assuming_equal_intrinsics(self):
        source = FakeSource()
        for index, (topic, message, log_ns) in enumerate(source.rows):
            if topic == source.topics["rgb_info"]:
                source.rows[index] = (topic, camera_info(10**10, 4, 4, 20.0), log_ns)
            elif topic == source.topics["rgb"]:
                rgb = np.zeros((4, 4, 3), dtype=np.uint8)
                rgb[0, 0] = [255, 0, 0]
                rgb[2, 0] = [0, 255, 0]
                source.rows[index] = (topic, image(rgb, 10_500_000_000, "rgb8"), log_ns)
        run = self.load(source)
        _, colours = run.cloud(run.frames[0])
        np.testing.assert_array_equal(colours, [[255, 0, 0], [0, 255, 0]])

    def test_bgr_images_are_converted_to_rgb(self):
        source = FakeSource()
        for index, (topic, message, log_ns) in enumerate(source.rows):
            if topic == source.topics["rgb"]:
                array = np.full((2, 2, 3), [10, 20, 30], dtype=np.uint8)
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

    def test_interpolation_holds_endpoints_but_rejects_interior_gaps(self):
        series = (np.array([100, 200]), np.array([0.0, 2.0]))
        self.assertEqual(sample_angle(series, 150, 100), 1.0)
        self.assertEqual(sample_angle(series, 50, 10), 0.0)
        self.assertEqual(sample_angle(series, 250, 10), 2.0)
        with self.assertRaisesRegex(ValueError, "gap"):
            sample_angle(series, 150, 99)

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
