"""Check wheel timing, encoder anchoring, reversals and missing telemetry."""

import numpy as np
import pytest

from legwheel_kinematics.encoder_yaw import EncoderYawTrajectory


def trajectory(encoder_angles, wheel_velocity, *, maximum_wheel_gap_s=0.05):
    seconds = np.arange(len(encoder_angles)) * 0.01
    times = 1_790_000_000_000_000_000 + np.rint(seconds * 1e9).astype(np.int64)
    return times, EncoderYawTrajectory(
        times,
        encoder_angles,
        times,
        wheel_velocity,
        maximum_wheel_gap_s=maximum_wheel_gap_s,
    )


def test_acceleration_uses_exact_partial_integrals_and_ignores_unchanged_messages():
    seconds = np.arange(101) * 0.01
    # Only the final encoder message changes. A query at an unchanged encoder
    # timestamp must still use wheel timing rather than pinning yaw to zero.
    angles = np.r_[np.zeros(100), 0.2]
    times, yaw = trajectory(angles, -(1.0 + 2.0 * seconds))
    for elapsed in (0.0, 0.25, 0.505, 0.75, 1.0):
        expected = 0.2 * (elapsed + elapsed**2) / 2.0
        assert yaw.sample(int(times[0]) + round(elapsed * 1e9)) == pytest.approx(
            expected, abs=1e-12
        )


def test_stops_and_reverse_ticks_are_retained_without_accumulating_wheel_slip():
    seconds = np.arange(401) * 0.01
    angles = np.select([seconds < 1, seconds < 3, seconds < 4], [0.0, 0.2, 0.1], 0.0)
    # Drive forward, pause, then reverse. Unequal wheel scale in each interval
    # represents slip; each central-encoder change must still be exact.
    velocity = np.interp(seconds, [0, 1, 1.01, 1.99, 2, 3, 4], [2, 2, 0, 0, -4, -4, -1])
    times, yaw = trajectory(angles, velocity)
    assert yaw.sample(int(times[100])) == pytest.approx(0.2)
    assert yaw.sample(int(times[300])) == pytest.approx(0.1)
    assert yaw.sample(int(times[400])) == pytest.approx(0.0)
    assert yaw.sample(int(times[150])) == pytest.approx(yaw.sample(int(times[190])))
    assert 0.1 < yaw.sample(int(times[250])) < 0.2
    assert 0.0 < yaw.sample(int(times[350])) < 0.1


def test_stationary_recording_does_not_move_with_wheel_spin():
    times, yaw = trajectory(np.full(101, -8.0), np.full(101, 20.0))
    assert yaw.sample(int(times[45]) + 123) == -8.0
    assert yaw.sample(int(times[-1])) == -8.0


def test_wheel_gaps_keep_encoder_pose_and_refinement_recovers():
    times = np.arange(201, dtype=np.int64) * 10_000_000
    angles = np.r_[np.zeros(100), np.ones(100), 2.0]
    velocity = np.ones(201)
    velocity[40] = np.nan
    yaw = EncoderYawTrajectory(times, angles, times, velocity)
    assert yaw.sample(500_000_000) == 0.0
    assert yaw.sample(1_500_000_000) == pytest.approx(1.5)
    for timestamp in (-1, int(times[-1]) + 1):
        with pytest.raises(ValueError, match="outside"):
            yaw.sample(timestamp)
    # A long central-encoder outage stays a gap even with perfect wheel data.
    yaw = EncoderYawTrajectory([0, 2_000_000_000], [0, 1], times, np.ones(201))
    with pytest.raises(ValueError, match="Encoder.*gap"):
        yaw.sample(1_000_000_000)
    assert yaw.sample(2_000_000_000) == 1.0
    # Sparse wheel telemetry cannot be made valid by inserting encoder knots;
    # those intervals use the encoder instead of fabricating wheel refinement.
    yaw = EncoderYawTrajectory(times, angles, times[[0, -1]], [1, 1])
    assert yaw.sample(500_000_000) == 0.0
    assert yaw.sample(995_000_000) == pytest.approx(0.5)


def test_inconsistent_zero_wheel_integral_does_not_produce_infinite_yaw():
    times, yaw = trajectory(np.r_[np.zeros(100), 0.2], np.zeros(101))
    assert yaw.sample(int(times[50])) == 0.0
    assert yaw.sample(int(times[99]) + 5_000_000) == pytest.approx(0.1)
    assert yaw.sample(int(times[-1])) == 0.2
