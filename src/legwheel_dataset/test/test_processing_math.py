import math
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np

from legwheel_dataset.features import (
    calculate_spectra,
    fit_plane_ransac,
    slope_from_plane,
)
from legwheel_dataset.config import ProcessingConfig
from legwheel_dataset.processor import extract_features
from legwheel_dataset.processor import isolate_packets
from legwheel_dataset.bag_reader import (
    BagIndex,
    CalibrationTimeline,
    ImageMetadata,
    TelemetrySeries,
)
from legwheel_kinematics.projection import CameraCalibration
from legwheel_dataset.synchronization import (
    find_future_crossing_time,
    pair_image_timestamps,
)


class SynchronizationTest(unittest.TestCase):
    def test_pairing_is_nearest_one_to_one_with_tolerance(self):
        rgb = np.array([100_000_000, 200_000_000, 300_000_000])
        depth = np.array([96_000_000, 207_000_000, 320_000_000])
        pairs = pair_image_timestamps(rgb, depth, maximum_delta_ns=10_000_000)
        self.assertEqual(len(pairs), 2)
        self.assertEqual(pairs[0].sync_error_ns, -4_000_000)
        self.assertEqual(pairs[1].sync_error_ns, 7_000_000)

    def test_contact_time_interpolates_encoder_crossing(self):
        times = np.array([0, 1_000_000_000, 2_000_000_000], dtype=np.int64)
        angles = np.array([0.0, 1.0, 2.0])
        crossing, direction = find_future_crossing_time(
            times,
            angles,
            image_time_ns=500_000_000,
            angle_offset_rad=0.75,
        )
        self.assertEqual(crossing, 1_250_000_000)
        self.assertEqual(direction, 1.0)

    def test_stage1_builds_packet_from_indexed_sensor_data(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_directory = Path(temporary_directory) / "run_1"
            (run_directory / "rosbag").mkdir(parents=True)
            experiment_config = {
                "rig_config": {
                    "pivot_height_m": 0.353,
                    "beam_radius_m": 0.83,
                },
                "leg_config": {
                    "hip_link_length_m": 0.2,
                    "calf_link_length_m": 0.2,
                },
                "electronics_hardware": {
                    "camera": {
                        "camera_config": {
                            "camera_beam_offset_m": -0.043,
                            "camera_height_m": 0.029,
                            "camera_front_angle_deg": 40.0,
                        }
                    }
                },
                "wheel_config": {
                    "wheel_radius_mm": 50.0,
                    "wheel_width_mm": 100.0,
                },
            }
            calibration = CameraCalibration(
                width=640,
                height=480,
                K=np.array([
                    [461.8, 0.0, 324.2],
                    [0.0, 461.7, 242.9],
                    [0.0, 0.0, 1.0],
                ]),
                D=np.zeros(5),
                distortion_model="plumb_bob",
            )
            sample_times = np.arange(0, 401, dtype=np.int64) * 10_000_000
            knee_joint = 1.0
            leg = TelemetrySeries(tuple(
                f"{name}.{field}"
                for name in ("hip_joint", "knee_joint")
                for field in ("position", "velocity", "effort")
            ))
            wheel = TelemetrySeries((
                "wheel_joint.position",
                "wheel_joint.velocity",
                "wheel_joint.effort",
            ))
            rotation = TelemetrySeries((
                "rotation_joint.position",
                "rotation_joint.velocity",
            ))
            imu = TelemetrySeries(tuple(f"imu.channel_{index}" for index in range(6)))
            for timestamp_ns in sample_times:
                time_s = timestamp_ns * 1e-9
                leg.append(timestamp_ns, [0.0, 0.0, 0.0, knee_joint, 0.0, 0.0])
                wheel.append(timestamp_ns, [time_s, 1.0, 0.5])
                rotation.append(timestamp_ns, [-time_s, -1.0])
                imu.append(timestamp_ns, [0.0] * 6)
            timeline = CalibrationTimeline()
            timeline.append(0, calibration)
            bag_index = BagIndex(
                rgb_images=[ImageMetadata(1_500_000_000, 640, 480, "rgb8")],
                depth_images=[ImageMetadata(1_504_000_000, 640, 480, "16UC1")],
                rgb_calibrations=timeline,
                depth_calibrations=timeline,
                streams={
                    "leg": leg,
                    "wheel": wheel,
                    "rotation": rotation,
                    "imu": imu,
                },
            )

            def provide_images(_bag_directory, requests):
                for (topic, _), callback in requests.items():
                    if topic.endswith("color/image_raw"):
                        array = np.zeros((480, 640, 3), dtype=np.uint8)
                        encoding = "rgb8"
                    else:
                        array = np.full((480, 640), 1000, dtype=np.uint16)
                        encoding = "16UC1"
                    callback(SimpleNamespace(
                        encoding=encoding,
                        width=640,
                        height=480,
                        is_bigendian=False,
                        step=array.strides[0],
                        data=array.tobytes(),
                    ))
                return set(requests)

            output_directory = run_directory / "derived_test"
            with (
                patch("legwheel_dataset.processor._load_yaml", return_value=experiment_config),
                patch("legwheel_dataset.processor.index_bag", return_value=bag_index),
                patch(
                    "legwheel_dataset.processor.extract_selected_images",
                    side_effect=provide_images,
                ),
            ):
                isolate_packets(
                    run_directory,
                    ProcessingConfig(),
                    experiment_config_path=run_directory / "experiment.yaml",
                    output_directory=output_directory,
                )

            manifest = (output_directory / "manifest.jsonl").read_text(
                encoding="utf-8"
            )
            self.assertIn("packet_000000", manifest)
            self.assertTrue(
                (output_directory / "packets" / "packet_000000" / "rgb.npy").is_file()
            )


class FeatureTest(unittest.TestCase):
    def test_plane_slope_is_measured_in_travel_direction(self):
        x, y = np.meshgrid(np.linspace(-1, 1, 20), np.linspace(-1, 1, 20))
        z = 0.2 * y
        points = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
        plane = fit_plane_ransac(points, 50, 0.001, 100)
        along, cross = slope_from_plane(
            plane.normal,
            midpoint_angle_rad=0.0,
            travel_direction=1.0,
        )
        self.assertAlmostEqual(along, math.atan(0.2), places=5)
        self.assertAlmostEqual(cross, 0.0, places=5)

    def test_spectrum_finds_eighteen_hertz_signal(self):
        rate_hz = 100.0
        times = np.arange(100) / rate_hz
        values = np.sin(2.0 * np.pi * 18.0 * times)[:, None]
        valid = np.ones(values.shape, dtype=bool)
        _, _, dominant = calculate_spectra(values, valid, rate_hz, 1.0)
        self.assertAlmostEqual(dominant[0], 18.0, places=6)

    def test_stage2_extracts_flat_plane_from_synthetic_packet(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            dataset = Path(temporary_directory)
            packet = dataset / "packets" / "packet_000000"
            packet.mkdir(parents=True)
            config = ProcessingConfig(
                minimum_plane_points=50,
                plane_ransac_iterations=20,
            )
            (dataset / "provenance.json").write_text(
                json.dumps({"processing_config": config.to_dict()}),
                encoding="utf-8",
            )
            (dataset / "manifest.jsonl").write_text(
                json.dumps({
                    "packet_id": "packet_000000",
                    "packet_path": "packets/packet_000000",
                })
                + "\n",
                encoding="utf-8",
            )
            polygon = [[5.0, 5.0], [44.0, 5.0], [44.0, 44.0], [5.0, 44.0]]
            calibration = {
                "width": 50,
                "height": 50,
                "K": [[100.0, 0.0, 25.0], [0.0, 100.0, 25.0], [0.0, 0.0, 1.0]],
                "D": [],
                "distortion_model": "",
            }
            (packet / "metadata.json").write_text(
                json.dumps({
                    "packet_id": "packet_000000",
                    "rgb_polygon_pixels": polygon,
                    "depth_polygon_pixels": polygon,
                    "depth_calibration": calibration,
                    "T_cam": np.eye(4).tolist(),
                    "track_midpoint_angle_rad": 0.0,
                    "travel_direction": 1.0,
                }),
                encoding="utf-8",
            )
            np.save(packet / "rgb.npy", np.zeros((50, 50, 3), dtype=np.uint8))
            np.save(packet / "depth.npy", np.full((50, 50), 1000, dtype=np.uint16))
            telemetry = np.sin(2.0 * np.pi * 18.0 * np.arange(100) / 100.0)[:, None]
            np.savez_compressed(
                packet / "telemetry.npz",
                spectral_values=telemetry,
                spectral_valid=np.ones(telemetry.shape, dtype=bool),
            )

            extract_features(dataset)

            features = json.loads(
                (packet / "stage2" / "features.json").read_text(encoding="utf-8")
            )
            self.assertTrue(features["valid"])
            self.assertAlmostEqual(features["along_slope_deg"], 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
