"""Check image-time association, impact barriers and bounded pose retention."""

import math
from types import SimpleNamespace

import pytest

from legwheel_kinematics.pose_history import PoseHistory, joint_state_pose


def test_image_time_is_interpolated_without_using_the_latest_pose():
    history = PoseHistory()
    history.append(1_000_000_000, -0.2, 0.1)
    with pytest.raises(ValueError, match="outside"):
        history.sample(1_005_000_000)
    history.append(1_010_000_000, -0.4, 0.3)
    assert history.sample(1_005_000_000) == pytest.approx((-0.3, 0.2))
    assert history.sample(1_010_000_000) == (-0.4, 0.3)
    for timestamp_ns in (999_000_000, 1_011_000_000):
        with pytest.raises(ValueError, match="outside"):
            history.sample(timestamp_ns)


def test_a_single_rejected_sample_prevents_interpolation_through_an_impact():
    history = PoseHistory(maximum_gap_s=0.05)
    history.append(1_000_000_000, 0.0, 0.0)
    history.append_unavailable(1_010_000_000)
    history.append(1_020_000_000, 0.0, 0.2)
    for timestamp_ns in (1_005_000_000, 1_010_000_000, 1_015_000_000):
        with pytest.raises(ValueError, match="rejected"):
            history.sample(timestamp_ns)
    assert history.sample(1_020_000_000) == (0.0, 0.2)


def test_missing_pose_intervals_are_not_bridged():
    history = PoseHistory(maximum_gap_s=0.05)
    history.append(1_000_000_000, 0.0, 0.0)
    history.append(1_060_000_000, 0.2, 0.2)
    with pytest.raises(ValueError, match="gap"):
        history.sample(1_030_000_000)


def test_late_delivery_duplicate_updates_and_history_limit_keep_times_consistent():
    history = PoseHistory(maximum_samples=3)
    history.append(1_020_000_000, 6.2, 0.2)
    history.append(1_000_000_000, 6.0, 0.0)
    history.append(1_010_000_000, 6.1, 0.1)
    history.append(1_010_000_000, 6.1, 0.15)
    assert history.sample(1_015_000_000) == pytest.approx((6.15, 0.175))
    history.append(1_030_000_000, 6.3, 0.3)
    with pytest.raises(ValueError, match="outside"):
        history.sample(1_000_000_000)
    assert history.newest_stamp_ns == 1_030_000_000
    history.clear()
    assert history.newest_stamp_ns is None
    with pytest.raises(ValueError, match="outside"):
        history.sample(1_030_000_000)


def test_named_imu_positions_are_independent_of_joint_order_and_never_use_knees():
    message = SimpleNamespace(
        name=["knee_joint", "theta_p_joint", "theta_y_joint"],
        position=[99.0, 0.15, -0.7],
    )
    assert joint_state_pose(message) == (-0.7, 0.15)
    message.name, message.position = ["knee_joint"], [0.15]
    with pytest.raises(ValueError, match="theta_p_joint"):
        joint_state_pose(message)
    message.name, message.position = ["theta_y_joint", "theta_p_joint"], [0.0, math.nan]
    with pytest.raises(ValueError, match="nonfinite"):
        joint_state_pose(message)
