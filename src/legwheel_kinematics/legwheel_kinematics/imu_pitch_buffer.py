"""A bounded visualization window for IMU pitch and wheel-refined encoder yaw.

ROS callbacks only collect measurements here. A slower visualization timer can
then call ``estimate`` to run the same filter and offline smoother used for a
complete dataset, with the benefit of a short delay for later observations.
This class does not introduce another orientation filter or use knee geometry.
"""

import math
import operator
from bisect import bisect_left
from typing import Any

import numpy as np

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.encoder_yaw import EncoderYawTrajectory
from legwheel_kinematics.imu_pitch import PitchTrajectory, estimate_pitch


class ImuPitchBuffer:
    """Keep recent IMU/encoder/wheel samples and stationary pitch calibration.

    Small source timestamp reorderings are sorted instead of mistaken for a
    playback seek. The ROS adapter must call ``clear`` on a backwards /clock
    jump; data from two playback epochs must never share a filter window.
    """

    def __init__(
        self,
        geometry: KinematicsConfig,
        *,
        window_s: float = 10.0,
        maximum_gap_s: float = 0.05,
        maximum_uncertainty_deg: float = 1.0,
        maximum_wheel_gap_s: float = 0.05,
        encoder_origin_motion_rad: float | None = None,
    ) -> None:
        for name, value in (
            ("window_s", window_s),
            ("maximum_gap_s", maximum_gap_s),
            ("maximum_uncertainty_deg", maximum_uncertainty_deg),
            ("maximum_wheel_gap_s", maximum_wheel_gap_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")
        if window_s < max(2.0, maximum_gap_s):
            raise ValueError("window_s must be at least two seconds and maximum_gap_s.")
        self.geometry = geometry
        self.window_s = float(window_s)
        self.maximum_gap_s = float(maximum_gap_s)
        self.maximum_uncertainty_deg = float(maximum_uncertainty_deg)
        self.maximum_wheel_gap_s = float(maximum_wheel_gap_s)
        # Playback supplies angles relative to the complete recording's origin.
        # Retain that origin when a seek clears the bounded sensor history.
        if encoder_origin_motion_rad is not None and not math.isfinite(
            encoder_origin_motion_rad
        ):
            raise ValueError("encoder_origin_motion_rad must be finite.")
        self._fixed_encoder_origin = encoder_origin_motion_rad
        self.clear()

    def clear(self) -> None:
        """Clear sensor history while preserving an explicit recording origin."""
        self._imu_times: list[int] = []
        self._imu_rows: list[tuple[np.ndarray, np.ndarray]] = []
        self._encoder_times: list[int] = []
        self._encoder_angles: list[float] = []
        self._wheel_times: list[int] = []
        self._wheel_velocities: list[float] = []
        self._yaw_trajectory: EncoderYawTrajectory | None = None
        self._newest_time: int | None = None
        self._yaw_origin_time: int | None = None
        self._yaw_origin_motion_angle: float | None = self._fixed_encoder_origin
        # Only actual stationary calibration is cached. Reusing a fitted moving
        # posterior on every overlapping window would count the same data twice.
        self._stationary_bias: tuple[float, float, int, float] | None = None

    @property
    def imu_times_ns(self) -> tuple[int, ...]:
        """Read-only timestamps for selecting delayed visualization samples."""
        return tuple(self._imu_times)

    @property
    def encoder_times_ns(self) -> tuple[int, ...]:
        """Read-only encoder timestamps retained in the same time window."""
        return tuple(self._encoder_times)

    @property
    def latest_time_ns(self) -> int | None:
        """Latest pose coverage once all three telemetry streams have arrived.

        Wheel gaps disable refinement rather than freezing central-encoder yaw.
        The publication delay normally lets wheel measurements catch up; an
        actual outage must not prevent newer encoder/IMU poses being displayed.
        """
        if not self._imu_times or not self._encoder_times or not self._wheel_times:
            return None
        return min(self._imu_times[-1], self._encoder_times[-1])

    @staticmethod
    def _timestamp(timestamp_ns: Any) -> int:
        try:
            timestamp = operator.index(timestamp_ns)
        except TypeError as error:
            raise ValueError(
                "Sensor timestamps must be integer nanoseconds."
            ) from error
        if isinstance(timestamp_ns, bool) or not -(2**63) <= timestamp < 2**63:
            raise ValueError("Sensor timestamps must fit signed 64-bit nanoseconds.")
        return timestamp

    def _too_old(self, timestamp_ns: int) -> bool:
        return self._newest_time is not None and timestamp_ns < (
            self._newest_time - round(self.window_s * 1e9)
        )

    def _prune(self, timestamp_ns: int) -> None:
        """A fast stream also expires stale samples from the other sensors."""
        self._newest_time = (
            timestamp_ns
            if self._newest_time is None
            else max(self._newest_time, timestamp_ns)
        )
        oldest_time = self._newest_time - round(self.window_s * 1e9)
        for times, values in (
            (self._imu_times, self._imu_rows),
            (self._encoder_times, self._encoder_angles),
            (self._wheel_times, self._wheel_velocities),
        ):
            first_retained = bisect_left(times, oldest_time)
            del times[:first_retained]
            del values[:first_retained]
        self._yaw_trajectory = None

    def add_imu(
        self,
        timestamp_ns: int,
        angular_velocity: Any,
        linear_acceleration: Any,
        frame_id: str = "",
    ) -> None:
        """Collect optical-axis gyro (rad/s) and specific force (m/s squared).

        Current and legacy Gemini camera optical frame names are accepted.
        Unknown body frames require an explicit transform upstream; assuming
        their axes would silently change both pitch sign and camera correction.
        Nonfinite vectors remain in history so the core marks a real gap.
        """
        timestamp = self._timestamp(timestamp_ns)
        if frame_id and not frame_id.endswith("optical_frame"):
            raise ValueError(
                f"IMU frame {frame_id!r} must use camera optical axes "
                "(an optical_frame suffix); transform body-frame IMU data first."
            )
        gyro = np.asarray(angular_velocity, dtype=float)
        acceleration = np.asarray(linear_acceleration, dtype=float)
        if gyro.shape != (3,) or acceleration.shape != (3,):
            raise ValueError(
                "IMU angular velocity and acceleration must be three-vectors."
            )
        if self._too_old(timestamp):
            return
        index = bisect_left(self._imu_times, timestamp)
        # Last receipt wins at a duplicate timestamp, as it can contain corrected
        # data. Copies prevent a ROS message/array reused by the caller changing
        # measurements already in this history.
        row = (gyro.copy(), acceleration.copy())
        if index < len(self._imu_times) and self._imu_times[index] == timestamp:
            self._imu_rows[index] = row
        else:
            self._imu_times.insert(index, timestamp)
            self._imu_rows.insert(index, row)
        self._prune(timestamp)

    def add_wheel(self, timestamp_ns: int, velocity_rad_s: float) -> None:
        """Collect signed wheel angular velocity, using its acquisition time.

        The yaw estimator normalizes the integrated velocity between encoder
        anchors. Motor sign and the effective rolling radius cancel there, so
        no extra wheel sign convention or geometry correction belongs here.
        """
        timestamp = self._timestamp(timestamp_ns)
        if self._too_old(timestamp):
            return
        index = bisect_left(self._wheel_times, timestamp)
        if index < len(self._wheel_times) and self._wheel_times[index] == timestamp:
            self._wheel_velocities[index] = float(velocity_rad_s)
        else:
            self._wheel_times.insert(index, timestamp)
            self._wheel_velocities.insert(index, float(velocity_rad_s))
        self._prune(timestamp)

    def add_encoder(self, timestamp_ns: int, angle_motion_rad: float) -> None:
        """Collect a central-encoder angle, before the configured yaw sign.

        The rotation-encoder node supplies continuous count-derived angles.
        Keep those full turns verbatim: shortest-path angle unwrapping would
        discard known revolutions after a long recording gap. Nonfinite counts
        stay as explicit data gaps.
        """
        timestamp = self._timestamp(timestamp_ns)
        angle = float(angle_motion_rad)
        if self._too_old(timestamp):
            return
        index = bisect_left(self._encoder_times, timestamp)
        if index < len(self._encoder_times) and self._encoder_times[index] == timestamp:
            self._encoder_angles[index] = angle
        else:
            self._encoder_times.insert(index, timestamp)
            self._encoder_angles.insert(index, angle)
        if self._fixed_encoder_origin is None and math.isfinite(angle) and (
            self._yaw_origin_time is None or timestamp <= self._yaw_origin_time
        ):
            self._yaw_origin_time = timestamp
            self._yaw_origin_motion_angle = angle
        self._prune(timestamp)

    def _yaw_angles(self) -> np.ndarray:
        """Apply the hardware-to-kinematics sign once and keep one yaw origin."""
        if self._yaw_origin_motion_angle is None:
            raise ValueError("No finite central encoder angle has been recorded.")
        return self.geometry.theta_y_kinematic_sign * (
            np.asarray(self._encoder_angles) - self._yaw_origin_motion_angle
        )

    def yaw_at(self, timestamp_ns: int) -> float:
        """Use the same encoder-anchored wheel refinement as full-run replay."""
        timestamp = self._timestamp(timestamp_ns)
        if self._yaw_trajectory is None:
            # Build once per sensor batch, rather than reintegrating the wheel
            # history separately for every IMU pose published by the timer.
            self._yaw_trajectory = EncoderYawTrajectory(
                self._encoder_times,
                self._yaw_angles(),
                self._wheel_times,
                self._wheel_velocities,
                maximum_encoder_gap_s=0.25,
                maximum_wheel_gap_s=self.maximum_wheel_gap_s,
            )
        return self._yaw_trajectory.sample(timestamp)

    def estimate(self) -> PitchTrajectory | None:
        """Run the shared estimator at visualization cadence, after collecting.

        The returned trajectory has its native IMU timestamps and reliability
        checks. Callers choose a slightly delayed timestamp and use ``sample``;
        they must not substitute a knee-derived pose when pitch is unavailable.
        """
        if len(self._imu_times) < 3 or len(self._encoder_times) < 3:
            return None
        prior = {}
        if self._stationary_bias is not None:
            bias, uncertainty, calibration_time, random_walk = self._stationary_bias
            age_s = max(0.0, (self._imu_times[0] - calibration_time) * 1e-9)
            # The remembered rest calibration becomes less informative as its
            # age grows; a new actual rest interval overrides this prior in core.
            prior = {
                "initial_gyro_bias_rad_s": bias,
                "initial_gyro_bias_uncertainty_rad_s": math.sqrt(
                    uncertainty**2 + random_walk**2 * age_s
                ),
            }
        gyro = np.asarray([row[0] for row in self._imu_rows])
        acceleration = np.asarray([row[1] for row in self._imu_rows])
        trajectory = estimate_pitch(
            np.asarray(self._imu_times, dtype=np.int64),
            gyro,
            acceleration,
            np.asarray(self._encoder_times, dtype=np.int64),
            self._yaw_angles(),
            self.geometry,
            maximum_gap_s=self.maximum_gap_s,
            maximum_uncertainty_deg=self.maximum_uncertainty_deg,
            **prior,
        )
        calibration_time = trajectory.report.get("stationary_calibration_end_time_ns")
        if calibration_time is not None and (
            self._stationary_bias is None
            or calibration_time >= self._stationary_bias[2]
        ):
            self._stationary_bias = (
                trajectory.report["initial_gyro_bias_rad_s"],
                trajectory.report["initial_gyro_bias_uncertainty_rad_s"],
                int(calibration_time),
                trajectory.report["gyro_bias_walk_rad_s_sqrt_s"],
            )
        return trajectory
