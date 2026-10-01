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
from legwheel_dataset.processor import (
    _select_midpoint_pair,
    validate_processing_configuration_with_preview,
)
from legwheel_dataset.bag_reader import (
    BagIndex,
    CalibrationTimeline,
    ImageMetadata,
    TelemetrySeries,
)
from legwheel_kinematics.projection import CameraCalibration
from legwheel_dataset.synchronization import (
    ImagePair,
    pair_image_timestamps,
)
from legwheel_dataset.run_analysis import (
    build_corrected_theta_trajectory,
    find_steady_torque_period,
)


class SynchronizationTest(unittest.TestCase):
    def test_pairing_is_nearest_one_to_one_with_tolerance(self):
        rgb = np.array([100_000_000, 200_000_000, 300_000_000])
        depth = np.array([96_000_000, 207_000_000, 320_000_000])
        pairs = pair_image_timestamps(rgb, depth, maximum_delta_ns=10_000_000)
        self.assertEqual(len(pairs), 2)
        self.assertEqual(pairs[0].sync_error_ns, -4_000_000)
        self.assertEqual(pairs[1].sync_error_ns, 7_000_000)

    def test_validation_pair_is_closest_to_steady_motion_midpoint(self):
        pairs = [
            ImagePair(0, 0, 90, 90),
            ImagePair(1, 1, 170, 172),
            ImagePair(2, 2, 203, 205),
            ImagePair(3, 3, 280, 282),
        ]

        selected = _select_midpoint_pair(
            pairs,
            steady_start_ns=100,
            steady_end_ns=300,
        )

        self.assertIs(selected, pairs[2])

    def test_stage1_builds_packet_from_indexed_sensor_data(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_directory = Path(temporary_directory) / "run_1"
            (run_directory / "rosbag").mkdir(parents=True)
            experiment_config = {
                "experiment_id": "synthetic_1",
                "rig_config": {
                    "pivot_height_m": 0.353,
                    "beam_radius_m": 0.83,
                },
                "leg_config": {
                    "hip_link_length_m": 0.2,
                    "calf_link_length_m": 0.2,
                },
                "electronics_hardware": {
                    "encoder_setup": {
                        "ticks_per_legwheel_revolution": 62.5,
                        "theta_y_kinematic_sign": -1,
                    },
                    "camera": {
                        "camera_config": {
                            "camera_beam_offset_m": -0.043,
                            "camera_height_m": 0.029,
                            # Keep the synthetic patch completely in frame so
                            # this test exercises packet writing, not rejection.
                            "camera_front_angle_deg": 60.0,
                        }
                    }
                },
                "wheel_config": {
                    "wheel_radius_mm": 50.0,
                    "wheel_width_mm": 100.0,
                },
            }
            (run_directory / "run_config.yaml").write_text(
                "wheel_parameters:\n  commanded_wheel_torque_nm: 1.0\n",
                encoding="utf-8",
            )
            (run_directory / "camera_driver_config_run_1_snapshot.yaml").write_text(
                "depth_registration: true\nalign_target_stream: COLOR\n",
                encoding="utf-8",
            )
            experiment_path = run_directory / "experiment.yaml"
            experiment_path.write_text(
                json.dumps(experiment_config),
                encoding="utf-8",
            )
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
            ticks = TelemetrySeries(("rotation_encoder.ticks",))
            torque_command = TelemetrySeries(("wheel.requested_torque",))
            imu = TelemetrySeries(tuple(f"imu.channel_{index}" for index in range(6)))
            for sample_index, timestamp_ns in enumerate(sample_times):
                time_s = timestamp_ns * 1e-9
                leg.append(timestamp_ns, [0.0, 0.0, 0.0, knee_joint, 0.0, 0.0])
                wheel.append(timestamp_ns, [time_s, 1.0, 0.5])
                encoder_ticks = math.floor(time_s * 10.0)
                # The encoder is intentionally only 20 Hz. It corrects theta_y
                # but must not limit the 100 Hz motor/IMU packet grid.
                if sample_index % 5 == 0:
                    ticks.append(timestamp_ns, [encoder_ticks])
                imu.append(timestamp_ns, [0.0] * 6)
            for timestamp_ns, torque in (
                (0, 0.0),
                (500_000_000, -0.95),
                (3_500_000_000, -1.0),
                (4_000_000_000, 0.0),
            ):
                torque_command.append(timestamp_ns, [torque])
            timeline = CalibrationTimeline()
            timeline.append(0, calibration, "camera_color_optical_frame")
            bag_index = BagIndex(
                rgb_images=[ImageMetadata(
                    1_500_000_000,
                    640,
                    480,
                    "rgb8",
                    "camera_color_optical_frame",
                )],
                depth_images=[ImageMetadata(
                    1_504_000_000,
                    640,
                    480,
                    "16UC1",
                    "camera_color_optical_frame",
                )],
                rgb_calibrations=timeline,
                depth_calibrations=timeline,
                streams={
                    "leg": leg,
                    "wheel": wheel,
                    "imu": imu,
                    "ticks": ticks,
                    "torque_command": torque_command,
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
                patch("legwheel_dataset.processor.index_bag", return_value=bag_index),
                patch(
                    "legwheel_dataset.processor.extract_selected_images",
                    side_effect=provide_images,
                ),
            ):
                validation_report, preview = (
                    validate_processing_configuration_with_preview(
                        run_directory,
                        ProcessingConfig(),
                        experiment_config_path=experiment_path,
                    )
                )
                isolate_packets(
                    run_directory,
                    ProcessingConfig(),
                    experiment_config_path=experiment_path,
                    output_directory=output_directory,
                )

            self.assertIsNotNone(preview)
            self.assertTrue(
                validation_report["visualization_preview"]["available"]
            )
            self.assertEqual(
                validation_report["visualization_preview"]["selection"],
                "synchronized_pair_closest_to_steady_motion_midpoint",
            )
            self.assertTrue(validation_report["configuration_valid"])
            self.assertLess(
                preview.patch_centre_kinematic_angle_rad
                - preview.rgb.theta_y_kinematic_rad,
                -0.3,
            )
            self.assertLess(
                float(np.max(np.abs(preview.rgb.patch.polygon_pixels))),
                10_000.0,
            )
            from legwheel_dataset.validation_plot import (
                create_validation_figure,
                load_preview_images,
            )

            with patch(
                "legwheel_dataset.validation_plot.extract_selected_images",
                side_effect=provide_images,
            ):
                preview_images, preview_error = load_preview_images(preview)
            self.assertIsNone(preview_error)
            self.assertEqual(preview_images["rgb"].shape, (480, 640, 3))
            self.assertEqual(preview_images["depth"].shape, (480, 640))

            import matplotlib

            matplotlib.use("Agg", force=True)
            import matplotlib.pyplot as plt

            figure = create_validation_figure(preview, preview_images)
            self.assertEqual(len(figure.axes), 4)
            self.assertIn("RGB camera window", figure.axes[0].get_title())
            self.assertIn("Depth expanded diagnostic", figure.axes[3].get_title())
            plt.close(figure)

            manifest = (output_directory / "manifest.jsonl").read_text(
                encoding="utf-8"
            )
            record = json.loads(manifest.strip())
            packet = output_directory / record["packet_path"]
            self.assertTrue(packet.name.startswith("esynthetic1_r1_theta_"))
            self.assertTrue((packet / "rgb.npy").is_file())
            self.assertTrue((packet / "depth.npy").is_file())
            self.assertTrue(
                (output_directory / "packets" / "analysis_report.json").is_file()
            )
            metadata = json.loads(
                (packet / "metadata.json").read_text(encoding="utf-8")
            )
            self.assertEqual(metadata["theta_y_kinematic_sign"], -1.0)
            self.assertEqual(metadata["travel_direction_motion"], 1.0)
            self.assertEqual(metadata["travel_direction_kinematic"], -1.0)
            self.assertAlmostEqual(
                metadata["theta_y_image_kinematic_rad"],
                -metadata["theta_y_image_motion_rad"],
            )
            self.assertAlmostEqual(
                metadata["patch_centre_kinematic_angle_rad"],
                -metadata["patch_centre_motion_angle_rad"],
            )
            self.assertEqual(len(metadata["telemetry_channel_names"]), 15)
            self.assertFalse(any(
                "rotation" in name
                for name in metadata["telemetry_channel_names"]
            ))
            telemetry = np.load(packet / "telemetry.npz", allow_pickle=False)
            self.assertEqual(telemetry["local_values"].shape[1], 15)
            self.assertEqual(telemetry["spectral_values"].shape[1], 15)
            analysis = json.loads(
                (
                    output_directory
                    / "packets"
                    / "analysis_report.json"
                ).read_text(encoding="utf-8")
            )
            self.assertEqual(analysis["rate_limit_violations"], [])
            self.assertNotIn("rotation", analysis["measured_effective_rates"])
            self.assertAlmostEqual(
                analysis["motion_reference_effective_rate"]["median_rate_hz"],
                20.0,
            )
            self.assertEqual(
                analysis["resampled_telemetry_channel_count"], 15
            )

            reverse_ticks = TelemetrySeries(("rotation_encoder.ticks",))
            for timestamp_ns, row in zip(ticks.times_ns, ticks.rows):
                reverse_ticks.append(timestamp_ns, [-row[0]])
            reverse_bag_index = BagIndex(
                rgb_images=bag_index.rgb_images,
                depth_images=bag_index.depth_images,
                rgb_calibrations=bag_index.rgb_calibrations,
                depth_calibrations=bag_index.depth_calibrations,
                streams={**bag_index.streams, "ticks": reverse_ticks},
            )
            reverse_output = run_directory / "derived_reverse_test"
            with patch(
                "legwheel_dataset.processor.index_bag",
                return_value=reverse_bag_index,
            ):
                reverse_report, reverse_preview = (
                    validate_processing_configuration_with_preview(
                        run_directory,
                        ProcessingConfig(),
                        experiment_config_path=experiment_path,
                    )
                )
                with self.assertRaisesRegex(
                    ValueError,
                    "Reverse recording rejected",
                ):
                    isolate_packets(
                        run_directory,
                        ProcessingConfig(),
                        experiment_config_path=experiment_path,
                        output_directory=reverse_output,
                    )

            self.assertIsNone(reverse_preview)
            self.assertFalse(reverse_report["configuration_valid"])
            self.assertFalse(
                reverse_report["corrected_motion"]["forward_recording"]
            )
            self.assertEqual(
                reverse_report["visualization_preview"]["reason"],
                "reverse_recording_rejected",
            )
            reverse_analysis = json.loads(
                (
                    reverse_output
                    / "packets"
                    / "analysis_report.json"
                ).read_text(encoding="utf-8")
            )
            self.assertEqual(
                reverse_analysis["status"],
                "failed_reverse_recording",
            )

    def test_encoder_correction_preserves_signed_tick_endpoints(self):
        torque_period = find_steady_torque_period(
            np.array([0, 1_000_000_000, 4_000_000_000], dtype=np.int64),
            np.array([0.0, -1.0, 0.0]),
            configured_target_nm=1.0,
            minimum_target_fraction=0.9,
        )
        times = np.arange(0, 41, dtype=np.int64) * 100_000_000
        encoder = -np.floor(np.arange(41) / 5.0) * 0.2
        trajectory = build_corrected_theta_trajectory(
            times,
            encoder,
            times,
            np.full(times.size, 2.0),
            torque_period,
            wheel_radius_m=0.1,
            grouser_effective_height_m=0.0,
            track_radius_m=0.8,
            maximum_interpolation_gap_ms=110.0,
        )
        self.assertTrue(np.all(trajectory.correction_factors < 0.0))
        for timestamp_ns, expected_theta in zip(
            trajectory.tick_times_ns,
            trajectory.theta_y_at_ticks_rad,
        ):
            self.assertAlmostEqual(
                trajectory.theta_y_at(int(timestamp_ns)),
                expected_theta,
                places=10,
            )
        crossing_time_ns, residual_rad = trajectory.first_future_crossing_time(
            image_time_ns=1_250_000_000,
            ahead_offset_rad=0.3,
        )
        self.assertEqual(crossing_time_ns, 2_000_000_000)
        self.assertAlmostEqual(residual_rad, 0.0, places=10)


class FeatureTest(unittest.TestCase):
    def test_plane_slope_is_measured_in_travel_direction(self):
        x, y = np.meshgrid(np.linspace(-1, 1, 20), np.linspace(-1, 1, 20))
        z = 0.2 * y
        points = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
        plane = fit_plane_ransac(points, 50, 0.001, 100)
        along, cross = slope_from_plane(
            plane.normal,
            patch_centre_angle_rad=0.0,
            travel_direction=1.0,
        )
        self.assertAlmostEqual(along, math.atan(0.2), places=5)
        self.assertAlmostEqual(cross, 0.0, places=5)

    def test_cross_slope_is_positive_when_terrain_rises_radially_outward(self):
        x, y = np.meshgrid(np.linspace(-1, 1, 20), np.linspace(-1, 1, 20))
        z = 0.1 * x
        points = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
        plane = fit_plane_ransac(points, 50, 0.001, 100)
        along, cross = slope_from_plane(
            plane.normal,
            patch_centre_angle_rad=0.0,
            travel_direction=1.0,
        )
        self.assertAlmostEqual(along, 0.0, places=5)
        self.assertAlmostEqual(cross, math.atan(0.1), places=5)

    def test_spectrum_finds_eighteen_hertz_signal(self):
        rate_hz = 100.0
        times = np.arange(100) / rate_hz
        values = np.sin(2.0 * np.pi * 18.0 * times)[:, None]
        valid = np.ones(values.shape, dtype=bool)
        _, _, dominant, dominant_power = calculate_spectra(
            values,
            valid,
            rate_hz,
            1.0,
            maximum_interpolation_gap_ms=50.0,
        )
        self.assertAlmostEqual(dominant[0, 0], 18.0, places=6)
        self.assertTrue(np.isfinite(dominant_power[0, 0]))

    def test_spectrum_outputs_two_peak_slots_for_all_fifteen_channels(self):
        rate_hz = 100.0
        times = np.arange(101) / rate_hz
        one_channel = (
            np.sin(2.0 * np.pi * 18.0 * times)
            + 0.4 * np.sin(2.0 * np.pi * 7.0 * times)
        )
        values = np.tile(one_channel[:, None], (1, 15))
        valid = np.ones(values.shape, dtype=bool)
        _, power, peaks, peak_power = calculate_spectra(
            values,
            valid,
            rate_hz,
            1.0,
            maximum_interpolation_gap_ms=50.0,
        )
        self.assertEqual(power.shape[1], 15)
        self.assertEqual(peaks.shape, (2, 15))
        self.assertEqual(peak_power.shape, (2, 15))

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
                    "depth_calibration": calibration,
                    "T_depth_camera_in_base": np.eye(4).tolist(),
                    "patch_centre_kinematic_angle_rad": 0.0,
                    "travel_direction_kinematic": 1.0,
                    "telemetry_channel_names": ["synthetic"],
                }),
                encoding="utf-8",
            )
            np.save(packet / "rgb.npy", np.zeros((50, 50, 3), dtype=np.uint8))
            np.save(packet / "depth.npy", np.full((50, 50), 1000, dtype=np.uint16))
            mask = np.zeros((50, 50), dtype=bool)
            mask[5:45, 5:45] = True
            np.savez_compressed(
                packet / "geometry.npz",
                rgb_mask=np.packbits(mask, axis=None),
                rgb_mask_shape=np.asarray(mask.shape, dtype=np.int64),
                depth_mask=np.packbits(mask, axis=None),
                depth_mask_shape=np.asarray(mask.shape, dtype=np.int64),
            )
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
            self.assertTrue((packet / "stage2" / "point_map.npz").is_file())


if __name__ == "__main__":
    unittest.main()
