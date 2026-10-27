"""Dataset and visualization yaw must share calibration, origin and refinement."""

import numpy as np
import pytest

from legwheel_dataset.run_analysis import (
    TorquePeriod,
    build_corrected_theta_trajectory,
)
from legwheel_kinematics.encoder_yaw import EncoderYawTrajectory


@pytest.mark.parametrize("encoder_sign", [-1.0, 1.0])
@pytest.mark.parametrize("wheel_scale", [-3.0, 2.0])
def test_visualization_matches_dataset_between_ticks_with_variable_wheel_speed(
    encoder_sign, wheel_scale
):
    epoch_ns = 1_790_000_000_000_000_000
    seconds = np.arange(401) * 0.01
    encoder_times = epoch_ns + np.rint(seconds * 1e9).astype(np.int64)
    # Convert the complete recording before selecting the dataset interval.
    raw_angles = np.floor(seconds / 0.4) * 0.2
    # Wheel and encoder samples deliberately have different timestamps. Testing
    # interior times also exercises partial trapezoids at both interval ends.
    wheel_seconds = np.arange(-0.013, 4.02, 0.007)
    wheel_times = epoch_ns + np.rint(wheel_seconds * 1e9).astype(np.int64)
    wheel_velocity = wheel_scale * (2.0 + wheel_seconds + np.sin(3 * wheel_seconds))
    period = TorquePeriod(int(encoder_times[0]), int(encoder_times[-1]), 1, 0.9, 1)
    dataset = build_corrected_theta_trajectory(
        encoder_times,
        raw_angles,
        wheel_times,
        wheel_velocity,
        period,
        wheel_radius_m=0.09,
        grouser_effective_height_m=0.012,
        track_radius_m=0.78,
        maximum_interpolation_gap_ms=50.0,
    )
    visual = EncoderYawTrajectory(
        encoder_times,
        encoder_sign * (raw_angles - raw_angles[0]),
        wheel_times,
        wheel_velocity,
    )
    # Selecting steady motion must not rebase the dataset to its first change.
    assert dataset.theta_y_at(dataset.start_time_ns) != 0.0
    query_times = (
        np.linspace(0, dataset.end_time_ns - dataset.start_time_ns, 137).astype(
            np.int64
        )
        + dataset.start_time_ns
    )
    for timestamp_ns in query_times:
        assert visual.sample(int(timestamp_ns)) == pytest.approx(
            encoder_sign * dataset.theta_y_at(int(timestamp_ns)), abs=1e-12
        )
