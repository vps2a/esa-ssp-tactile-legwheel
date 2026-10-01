"""Run-level torque, encoder, motion, and acquisition-rate analysis.

The functions in this module operate only on NumPy arrays.  Keeping this
physics separate from ROS and file I/O makes the slip correction testable and
ensures that Stage 1 and the configuration validator use identical maths.
"""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray


IntArray = NDArray[np.int64]
FloatArray = NDArray[np.float64]


def interpolate_scalar(
    times_ns: IntArray,
    values: FloatArray,
    target_time_ns: int,
) -> float:
    """Linearly interpolate one finite scalar without losing nanosecond precision."""
    right = int(np.searchsorted(times_ns, target_time_ns, side="left"))
    if right < times_ns.size and int(times_ns[right]) == int(target_time_ns):
        return float(values[right])
    if right == 0 or right == times_ns.size:
        raise ValueError("Target time lies outside the sampled range.")
    left = right - 1
    interval_ns = int(times_ns[right]) - int(times_ns[left])
    fraction = (int(target_time_ns) - int(times_ns[left])) / interval_ns
    return float(values[left] + fraction * (values[right] - values[left]))


def integrate_piecewise_linear(
    times_ns: IntArray,
    values: FloatArray,
    start_time_ns: int,
    end_time_ns: int,
    maximum_gap_ns: int,
) -> float:
    """Integrate a linearly interpolated signal with the trapezoidal rule."""
    if end_time_ns <= start_time_ns:
        raise ValueError("Integration end must be after its start.")
    if start_time_ns < times_ns[0] or end_time_ns > times_ns[-1]:
        raise ValueError("integration_outside_wheel_telemetry")

    first_right = int(np.searchsorted(times_ns, start_time_ns, side="right"))
    last_right = int(np.searchsorted(times_ns, end_time_ns, side="left"))
    first_left = max(0, first_right - 1)
    last_right = min(times_ns.size - 1, last_right)
    covering_times = times_ns[first_left:last_right + 1]
    if covering_times.size < 2:
        raise ValueError("insufficient_wheel_telemetry_for_integration")
    if np.any(np.diff(covering_times) > maximum_gap_ns):
        raise ValueError("wheel_telemetry_gap_too_large")

    interior = times_ns[(times_ns > start_time_ns) & (times_ns < end_time_ns)]
    knot_times = np.concatenate(
        (
            np.asarray([start_time_ns], dtype=np.int64),
            interior,
            np.asarray([end_time_ns], dtype=np.int64),
        )
    )
    knot_values = np.asarray(
        [interpolate_scalar(times_ns, values, int(time)) for time in knot_times],
        dtype=float,
    )
    if not np.all(np.isfinite(knot_values)):
        raise ValueError("nonfinite_wheel_velocity_in_integration")
    intervals_s = np.diff(knot_times).astype(float) * 1e-9
    return float(np.sum(0.5 * (knot_values[:-1] + knot_values[1:]) * intervals_s))


@dataclass(frozen=True)
class TorquePeriod:
    """The requested-torque interval accepted for dataset generation."""

    start_time_ns: int
    end_time_ns: int
    configured_target_nm: float
    acceptance_threshold_nm: float
    held_command_at_start_nm: float


def find_steady_torque_period(
    command_times_ns: IntArray,
    commanded_torque_nm: FloatArray,
    configured_target_nm: float,
    minimum_target_fraction: float,
) -> TorquePeriod:
    """Find the period during which command magnitude remains above 90% by default."""
    command_times_ns = np.asarray(command_times_ns, dtype=np.int64)
    commands = np.asarray(commanded_torque_nm, dtype=float)
    finite = np.isfinite(commands)
    command_times_ns = command_times_ns[finite]
    commands = commands[finite]
    if command_times_ns.size < 2:
        raise ValueError("too_few_requested_torque_samples")
    if configured_target_nm <= 0.0:
        raise ValueError("commanded_wheel_torque_nm_must_be_positive")

    threshold = float(configured_target_nm * minimum_target_fraction)
    accepted = np.abs(commands) >= threshold
    accepted_indices = np.flatnonzero(accepted)
    if accepted_indices.size == 0:
        raise ValueError("requested_torque_never_reached_threshold")
    start_index = int(accepted_indices[0])

    later_below = np.flatnonzero(~accepted[start_index + 1:])
    if later_below.size == 0:
        raise ValueError("requested_torque_never_left_steady_period")
    end_index = start_index + 1 + int(later_below[0])
    return TorquePeriod(
        start_time_ns=int(command_times_ns[start_index]),
        end_time_ns=int(command_times_ns[end_index]),
        configured_target_nm=float(configured_target_nm),
        acceptance_threshold_nm=threshold,
        held_command_at_start_nm=float(commands[start_index]),
    )


@dataclass(frozen=True)
class CorrectedThetaTrajectory:
    """Continuous encoder-constrained rig angle over complete tick intervals."""

    tick_times_ns: IntArray
    raw_encoder_angles_rad: FloatArray
    theta_y_at_ticks_rad: FloatArray
    correction_factors: FloatArray
    predicted_interval_angles_rad: FloatArray
    encoder_interval_angles_rad: FloatArray
    wheel_times_ns: IntArray
    wheel_velocity_rad_s: FloatArray
    radius_ratio: float
    maximum_gap_ns: int
    travel_direction: float
    rejected_reverse_transition_count: int

    @property
    def start_time_ns(self) -> int:
        return int(self.tick_times_ns[0])

    @property
    def end_time_ns(self) -> int:
        return int(self.tick_times_ns[-1])

    def theta_y_at(self, timestamp_ns: int) -> float:
        """Evaluate corrected theta_y at an arbitrary time inside the trajectory."""
        if timestamp_ns < self.start_time_ns or timestamp_ns > self.end_time_ns:
            raise ValueError("timestamp_outside_corrected_theta_range")
        if timestamp_ns == self.end_time_ns:
            return float(self.theta_y_at_ticks_rad[-1])
        interval = int(
            np.searchsorted(self.tick_times_ns, timestamp_ns, side="right")
        ) - 1
        interval = max(0, min(interval, self.correction_factors.size - 1))
        partial_wheel_angle = integrate_piecewise_linear(
            self.wheel_times_ns,
            self.wheel_velocity_rad_s,
            int(self.tick_times_ns[interval]),
            int(timestamp_ns),
            self.maximum_gap_ns,
        ) if timestamp_ns > int(self.tick_times_ns[interval]) else 0.0
        return float(
            self.theta_y_at_ticks_rad[interval]
            + self.correction_factors[interval]
            * self.radius_ratio
            * partial_wheel_angle
        )

    def first_future_crossing_time(
        self,
        image_time_ns: int,
        ahead_offset_rad: float,
    ) -> tuple[int, float]:
        """Return the first future time at a positive direction-relative offset."""
        if ahead_offset_rad <= 0.0:
            raise ValueError("alpha_off_rad_must_be_positive")
        image_theta = self.theta_y_at(image_time_ns)
        target_theta = image_theta + self.travel_direction * ahead_offset_rad

        interval = int(
            np.searchsorted(self.tick_times_ns, image_time_ns, side="right")
        ) - 1
        interval = max(0, interval)
        lower_time = int(image_time_ns)
        for index in range(interval, self.correction_factors.size):
            upper_time = int(self.tick_times_ns[index + 1])
            upper_theta = float(self.theta_y_at_ticks_rad[index + 1])
            crossed = (
                upper_theta >= target_theta
                if self.travel_direction > 0.0
                else upper_theta <= target_theta
            )
            if crossed:
                # theta_y is continuous within an interval. Bisection avoids
                # quantising the contact time to encoder or wheel timestamps.
                low = lower_time
                high = upper_time
                for _ in range(60):
                    if high - low <= 1:
                        break
                    middle = (low + high) // 2
                    middle_theta = self.theta_y_at(middle)
                    before = (
                        middle_theta < target_theta
                        if self.travel_direction > 0.0
                        else middle_theta > target_theta
                    )
                    if before:
                        low = middle
                    else:
                        high = middle
                crossing_time = high
                residual = self.theta_y_at(crossing_time) - target_theta
                return crossing_time, float(residual)
            lower_time = upper_time
        raise ValueError("patch_not_reached_before_corrected_run_end")


def _encoder_change_events(
    times_ns: IntArray,
    angles_rad: FloatArray,
    start_time_ns: int,
    end_time_ns: int,
) -> tuple[IntArray, FloatArray, float, int]:
    """Extract monotonic encoder changes and reject isolated reverse changes."""
    finite = np.isfinite(angles_rad)
    times_ns = np.asarray(times_ns, dtype=np.int64)[finite]
    angles_rad = np.unwrap(np.asarray(angles_rad, dtype=float)[finite])
    if times_ns.size < 2:
        raise ValueError("too_few_rotation_encoder_samples")

    changed = np.concatenate(([False], np.abs(np.diff(angles_rad)) > 1e-12))
    event_times = times_ns[changed]
    event_angles = angles_rad[changed]
    inside = (event_times >= start_time_ns) & (event_times <= end_time_ns)
    event_times = event_times[inside]
    event_angles = event_angles[inside]
    if event_times.size < 2:
        raise ValueError("too_few_encoder_ticks_during_steady_torque")

    delta_signs = np.sign(np.diff(event_angles))
    nonzero_signs = delta_signs[delta_signs != 0.0]
    direction = float(np.sign(np.median(nonzero_signs)))
    if direction == 0.0:
        raise ValueError("cannot_determine_encoder_travel_direction")

    kept_times = [int(event_times[0])]
    kept_angles = [float(event_angles[0])]
    rejected = 0
    for timestamp_ns, angle_rad in zip(event_times[1:], event_angles[1:]):
        change = float(angle_rad) - kept_angles[-1]
        if change * direction > 1e-12:
            kept_times.append(int(timestamp_ns))
            kept_angles.append(float(angle_rad))
        else:
            rejected += 1
    if len(kept_times) < 2:
        raise ValueError("too_few_monotonic_encoder_ticks")
    return (
        np.asarray(kept_times, dtype=np.int64),
        np.asarray(kept_angles, dtype=float),
        direction,
        rejected,
    )


def build_corrected_theta_trajectory(
    encoder_times_ns: IntArray,
    encoder_angles_rad: FloatArray,
    wheel_times_ns: IntArray,
    wheel_velocity_rad_s: FloatArray,
    torque_period: TorquePeriod,
    wheel_radius_m: float,
    grouser_effective_height_m: float,
    track_radius_m: float,
    maximum_interpolation_gap_ms: float,
) -> CorrectedThetaTrajectory:
    """Build theta_y(t) from signed wheel integration constrained by each tick."""
    tick_times, raw_tick_angles, direction, rejected = _encoder_change_events(
        encoder_times_ns,
        encoder_angles_rad,
        torque_period.start_time_ns,
        torque_period.end_time_ns,
    )
    finite_wheel = np.isfinite(wheel_velocity_rad_s)
    wheel_times = np.asarray(wheel_times_ns, dtype=np.int64)[finite_wheel]
    wheel_velocity = np.asarray(wheel_velocity_rad_s, dtype=float)[finite_wheel]
    if wheel_times.size < 2:
        raise ValueError("too_few_wheel_velocity_samples")

    radius_ratio = (
        float(wheel_radius_m) + float(grouser_effective_height_m)
    ) / float(track_radius_m)
    if radius_ratio <= 0.0 or not math.isfinite(radius_ratio):
        raise ValueError("wheel_to_track_radius_ratio_must_be_positive")
    maximum_gap_ns = int(round(maximum_interpolation_gap_ms * 1_000_000.0))

    encoder_deltas = np.diff(raw_tick_angles)
    predicted_deltas = []
    correction_factors = []
    for index, encoder_delta in enumerate(encoder_deltas):
        wheel_integral = integrate_piecewise_linear(
            wheel_times,
            wheel_velocity,
            int(tick_times[index]),
            int(tick_times[index + 1]),
            maximum_gap_ns,
        )
        predicted_delta = radius_ratio * wheel_integral
        if abs(predicted_delta) <= 1e-12:
            raise ValueError("zero_predicted_angle_between_encoder_ticks")
        predicted_deltas.append(predicted_delta)
        correction_factors.append(float(encoder_delta / predicted_delta))

    theta_y_at_ticks = raw_tick_angles - raw_tick_angles[0]
    return CorrectedThetaTrajectory(
        tick_times_ns=tick_times,
        raw_encoder_angles_rad=raw_tick_angles,
        theta_y_at_ticks_rad=theta_y_at_ticks,
        correction_factors=np.asarray(correction_factors, dtype=float),
        predicted_interval_angles_rad=np.asarray(predicted_deltas, dtype=float),
        encoder_interval_angles_rad=np.asarray(encoder_deltas, dtype=float),
        wheel_times_ns=wheel_times,
        wheel_velocity_rad_s=wheel_velocity,
        radius_ratio=radius_ratio,
        maximum_gap_ns=maximum_gap_ns,
        travel_direction=direction,
        rejected_reverse_transition_count=rejected,
    )


def time_weighted_mean_absolute_speed(
    times_ns: IntArray,
    velocity_rad_s: FloatArray,
    start_time_ns: int,
    end_time_ns: int,
    maximum_gap_ns: int,
) -> float:
    """Return integral(abs(velocity))/duration over the corrected run."""
    times = np.asarray(times_ns, dtype=np.int64)
    velocity = np.asarray(velocity_rad_s, dtype=float)
    if start_time_ns < times[0] or end_time_ns > times[-1]:
        raise ValueError("speed_average_outside_wheel_telemetry")
    # Reuse the normal integrator's strict source-coverage and finite-value
    # checks before handling absolute-value zero crossings below.
    integrate_piecewise_linear(
        times,
        velocity,
        start_time_ns,
        end_time_ns,
        maximum_gap_ns,
    )
    interior = times[(times > start_time_ns) & (times < end_time_ns)]
    knot_times = np.concatenate(
        (
            np.asarray([start_time_ns], dtype=np.int64),
            interior,
            np.asarray([end_time_ns], dtype=np.int64),
        )
    )
    if np.any(np.diff(knot_times) > maximum_gap_ns):
        raise ValueError("wheel_telemetry_gap_too_large")
    knot_velocity = np.asarray(
        [interpolate_scalar(times, velocity, int(time)) for time in knot_times]
    )
    integral = 0.0
    for index, duration_ns in enumerate(np.diff(knot_times)):
        first = float(knot_velocity[index])
        second = float(knot_velocity[index + 1])
        duration_s = float(duration_ns) * 1e-9
        if first * second >= 0.0:
            integral += 0.5 * (abs(first) + abs(second)) * duration_s
        else:
            first_fraction = abs(first) / (abs(first) + abs(second))
            integral += 0.5 * abs(first) * duration_s * first_fraction
            integral += 0.5 * abs(second) * duration_s * (1.0 - first_fraction)
    return float(integral / ((end_time_ns - start_time_ns) * 1e-9))


def effective_rate_statistics(
    times_ns: IntArray,
    start_time_ns: int,
    end_time_ns: int,
) -> dict[str, float | int | None]:
    """Describe the actual message rate measured inside the accepted run."""
    times = np.asarray(times_ns, dtype=np.int64)
    times = times[(times >= start_time_ns) & (times <= end_time_ns)]
    if times.size < 2:
        return {
            "sample_count": int(times.size),
            "median_rate_hz": None,
            "median_period_ms": None,
            "p95_period_ms": None,
            "minimum_interval_rate_hz": None,
            "maximum_interval_rate_hz": None,
        }
    intervals_s = np.diff(times).astype(float) * 1e-9
    return {
        "sample_count": int(times.size),
        "median_rate_hz": float(1.0 / np.median(intervals_s)),
        "median_period_ms": float(np.median(intervals_s) * 1000.0),
        "p95_period_ms": float(np.percentile(intervals_s, 95.0) * 1000.0),
        "minimum_interval_rate_hz": float(1.0 / np.max(intervals_s)),
        "maximum_interval_rate_hz": float(1.0 / np.min(intervals_s)),
    }
