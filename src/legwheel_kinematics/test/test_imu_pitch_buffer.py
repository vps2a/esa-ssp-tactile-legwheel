"""The streaming visualizer window must preserve the offline pitch convention."""

import math
import unittest
from dataclasses import replace

import numpy as np
from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.imu_pitch import GRAVITY_M_S2, estimate_pitch
from legwheel_kinematics.imu_pitch_buffer import ImuPitchBuffer
from legwheel_kinematics.transforms import frame_coordinates


def _geometry():
    return KinematicsConfig(
        d1=0.353,
        a_b=0.83,
        h_b=0.0661,
        sensor_front_offset=0.043,
        camera_height=0.029,
        camera_beam_offset=0.054,
        camera_angle_deg=57.0,
        Beam_radius=0.83,
        Wheel_width=0.12,
        wheel_radius_mm=110.0,
        hip_link_length_m=0.20,
        calf_link_length_m=0.20,
        theta_y_kinematic_sign=-1.0,
    )


def _measurements(seconds, pitch, yaw, geometry, bias_rad_s=0.0):
    """Differentiate world camera positions, then express motion in its axes."""
    pitch = np.broadcast_to(pitch, seconds.shape)
    transforms = np.asarray(
        [
            frame_coordinates(geometry, float(pitch_value), float(yaw_value))[-1]
            for pitch_value, yaw_value in zip(pitch, yaw)
        ]
    )
    positions = transforms[:, :3, 3]
    camera_to_world = transforms[:, :3, :3]
    specific_force_world = np.gradient(
        np.gradient(positions, seconds, axis=0, edge_order=2),
        seconds,
        axis=0,
        edge_order=2,
    )
    specific_force_world[:, 2] += GRAVITY_M_S2
    pitch_rate = np.gradient(pitch, seconds, edge_order=2)
    yaw_rate = np.gradient(yaw, seconds, edge_order=2)
    omega_world = np.column_stack(
        (
            np.sin(yaw) * (pitch_rate + bias_rad_s),
            -np.cos(yaw) * (pitch_rate + bias_rad_s),
            yaw_rate,
        )
    )
    gyro = np.einsum("nji,nj->ni", camera_to_world, omega_world)
    acceleration = np.einsum("nji,nj->ni", camera_to_world, specific_force_world)
    timestamps = np.rint(seconds * 1e9).astype(np.int64)
    return timestamps, gyro, acceleration


class ImuPitchBufferTest(unittest.TestCase):
    def test_same_estimator_camera_angle_and_yaw_sign_with_reordered_duplicates(self):
        geometry = _geometry()
        seconds = np.arange(0.0, 2.01, 0.01)
        yaw = -0.63 * seconds
        timestamps, gyro, acceleration = _measurements(seconds, -0.12, yaw, geometry)
        buffer = ImuPitchBuffer(geometry)
        self.assertIsNone(buffer.estimate())
        self.assertIsNone(buffer.latest_time_ns)
        indices = [0, 2, 1, *range(3, len(timestamps))]
        for index in indices:
            buffer.add_imu(
                timestamps[index],
                gyro[index],
                acceleration[index],
                frame_id="camera_gyro_optical_frame",
            )
            buffer.add_encoder(
                timestamps[index], 1.2 + yaw[index] / geometry.theta_y_kinematic_sign
            )
        # Repeated stamps replace their values rather than growing history.
        buffer.add_imu(timestamps[20], gyro[20], acceleration[20])
        buffer.add_encoder(timestamps[20], 1.2 + yaw[20] / -1.0)
        self.assertEqual(buffer.imu_times_ns, tuple(timestamps))
        self.assertEqual(buffer.encoder_times_ns, tuple(timestamps))
        self.assertEqual(buffer.latest_time_ns, timestamps[-1])
        trajectory = buffer.estimate()
        direct = estimate_pitch(
            timestamps, gyro, acceleration, timestamps, yaw, geometry
        )
        np.testing.assert_allclose(
            trajectory.theta_p_rad, direct.theta_p_rad, atol=1e-12, equal_nan=True
        )
        self.assertAlmostEqual(
            trajectory.sample(1_000_000_000), -0.12, delta=math.radians(0.03)
        )
        self.assertAlmostEqual(buffer.yaw_at(1_005_000_000), -0.63315)

    def test_history_is_bounded_without_changing_first_encoder_origin(self):
        geometry = _geometry()
        seconds = np.arange(0.0, 5.01, 0.01)
        yaw = -0.70 * seconds
        timestamps, gyro, acceleration = _measurements(seconds, 0.1, yaw, geometry)
        buffer = ImuPitchBuffer(geometry, window_s=2.0)
        for index, timestamp in enumerate(timestamps):
            buffer.add_encoder(timestamp, 2.0 - yaw[index])
            buffer.add_imu(timestamp, gyro[index], acceleration[index])
        self.assertEqual(len(buffer.imu_times_ns), 201)
        self.assertEqual(len(buffer.encoder_times_ns), 201)
        self.assertEqual(buffer.imu_times_ns[0], 3_000_000_000)
        self.assertAlmostEqual(buffer.yaw_at(4_000_000_000), -2.8)
        # A late old message cannot reintroduce the expired recording prefix.
        buffer.add_encoder(0, 0.0)
        buffer.add_imu(0, [0.0] * 3, [0.0] * 3)
        self.assertEqual(buffer.imu_times_ns[0], 3_000_000_000)
        self.assertAlmostEqual(buffer.yaw_at(4_000_000_000), -2.8)
        self.assertAlmostEqual(
            buffer.estimate().sample(4_000_000_000), 0.1, delta=math.radians(0.03)
        )

    def test_encoder_full_turns_remain_after_original_turn_expires(self):
        buffer = ImuPitchBuffer(_geometry(), window_s=2.0)
        for index in range(801):
            motion_angle = 1.2 * index * 0.01
            buffer.add_encoder(index * 10_000_000, motion_angle)
        self.assertAlmostEqual(buffer.yaw_at(7_500_000_000), -9.0)
        self.assertEqual(buffer.encoder_times_ns[0], 6_000_000_000)

        # The count-derived encoder also knows full turns after missing data;
        # keep that observation, but do not interpolate through the gap itself.
        buffer = ImuPitchBuffer(_geometry())
        for timestamp, angle in (
            (0, 2.0),
            (10_000_000, 2.1),
            (20_000_000, 2.2),
            (5_000_000_000, 18.0),
            (5_010_000_000, 18.1),
            (5_020_000_000, 18.2),
        ):
            buffer.add_encoder(timestamp, angle)
        self.assertAlmostEqual(buffer.yaw_at(5_000_000_000), -16.0)
        with self.assertRaisesRegex(ValueError, "gap"):
            buffer.yaw_at(2_000_000_000)

    def test_clear_resets_calibration_histories_and_encoder_origin_for_clock_rewind(
        self,
    ):
        buffer = ImuPitchBuffer(_geometry())
        buffer.add_encoder(5_000_000_000, 8.0)
        buffer.add_imu(5_000_000_000, [0.0] * 3, [0.0] * 3)
        buffer.clear()
        self.assertIsNone(buffer.estimate())
        self.assertIsNone(buffer.latest_time_ns)
        self.assertEqual(buffer.imu_times_ns, ())
        self.assertEqual(buffer.encoder_times_ns, ())
        buffer.add_encoder(0, 2.0)
        buffer.add_encoder(10_000_000, 2.1)
        self.assertAlmostEqual(buffer.yaw_at(10_000_000), -0.1)

    def test_gaps_nonfinite_measurements_and_unknown_frames_are_not_hidden(self):
        geometry = _geometry()
        seconds = np.concatenate(
            (np.arange(0.0, 1.01, 0.01), np.arange(1.5, 3.01, 0.01))
        )
        yaw = np.zeros_like(seconds)
        timestamps, gyro, acceleration = _measurements(seconds, 0.1, yaw, geometry)
        buffer = ImuPitchBuffer(geometry)
        for index, timestamp in enumerate(timestamps):
            buffer.add_encoder(timestamp, 0.0)
            buffer.add_imu(timestamp, gyro[index], acceleration[index])
        with self.assertRaisesRegex(ValueError, "gap"):
            buffer.yaw_at(1_200_000_000)
        with self.assertRaisesRegex(ValueError, "outside"):
            buffer.yaw_at(-1)
        with self.assertRaisesRegex(ValueError, "gap"):
            buffer.estimate().sample(1_200_000_000, require_reliable=False)
        buffer.add_encoder(2_000_000_000, float("nan"))
        buffer.add_imu(2_000_000_000, [float("nan"), 0.0, 0.0], [0.0] * 3)
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            buffer.yaw_at(2_000_000_000)
        with self.assertRaisesRegex(ValueError, "gap"):
            buffer.estimate().sample(2_000_000_000, require_reliable=False)
        with self.assertRaisesRegex(ValueError, "optical"):
            buffer.add_imu(3_100_000_000, [0.0] * 3, [0.0] * 3, "camera_link")
        with self.assertRaisesRegex(ValueError, "three-vectors"):
            buffer.add_imu(3_100_000_000, [0.0], [0.0] * 3)
        with self.assertRaisesRegex(ValueError, "integer"):
            buffer.add_encoder(1.5, 0.0)
        for keyword in (
            {"window_s": 1.0},
            {"window_s": math.nan},
            {"maximum_gap_s": 0.0},
            {"maximum_uncertainty_deg": math.inf},
        ):
            with self.assertRaises(ValueError):
                ImuPitchBuffer(geometry, **keyword)

    def test_long_motion_survives_window_eviction_with_bias_and_repeated_impacts(self):
        geometry = replace(_geometry(), camera_angle_deg=-28.0)
        seconds = np.arange(0.0, 25.01, 0.01)
        moving_time = np.maximum(0.0, seconds - 2.0)
        pitch = 0.08 + 0.04 * (1.0 - np.cos(0.55 * moving_time))
        yaw_rate = -0.62 * np.minimum(moving_time, 1.0)
        yaw = np.cumsum(yaw_rate) * 0.01
        gyro_bias = math.radians(1.0)
        timestamps, gyro, acceleration = _measurements(
            seconds, pitch, yaw, geometry, bias_rad_s=gyro_bias
        )
        impact = (seconds > 4.0) & (np.mod(seconds - 4.0, 3.0) < 0.15)
        acceleration[impact] += [6.0, -12.0, 8.0]
        buffer = ImuPitchBuffer(geometry)
        retained_errors = []
        after_eviction = []
        for index, timestamp in enumerate(timestamps):
            buffer.add_encoder(timestamp, -yaw[index])
            buffer.add_imu(timestamp, gyro[index], acceleration[index])
            # A timer, not each sensor callback, runs the shared batch smoother.
            if index < 100 or index % 50:
                continue
            trajectory = buffer.estimate()
            delayed_time = timestamp - 400_000_000
            try:
                estimate = trajectory.sample(delayed_time)
            except ValueError:
                continue  # Impacts intentionally make these frames unavailable.
            expected = pitch[index - 40]
            retained_errors.append(abs(estimate - expected))
            self.assertAlmostEqual(buffer.yaw_at(delayed_time), yaw[index - 40])
            if seconds[index] > 13.0:
                after_eviction.append(trajectory.report["gyro_bias_source"])
        self.assertGreater(len(retained_errors), 30)
        self.assertGreater(len(after_eviction), 15)
        self.assertEqual(set(after_eviction), {"previous_stationary_calibration"})
        self.assertLess(max(retained_errors), math.radians(0.5))
        self.assertEqual(len(buffer.imu_times_ns), 1001)
        self.assertAlmostEqual(
            trajectory.report["initial_gyro_bias_rad_s"],
            gyro_bias,
            delta=math.radians(0.01),
        )
        self.assertGreater(
            trajectory.report["initial_gyro_bias_uncertainty_rad_s"],
            math.radians(0.05),
        )
        # A clock seek must discard the remembered calibration as well as data.
        buffer.clear()
        for index in range(len(timestamps) - 201, len(timestamps)):
            buffer.add_encoder(timestamps[index], -yaw[index])
            buffer.add_imu(timestamps[index], gyro[index], acceleration[index])
        self.assertEqual(buffer.estimate().report["gyro_bias_source"], "estimated")


if __name__ == "__main__":
    unittest.main()
