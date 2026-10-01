"""Interpolation helpers for clock-driven dataset frame playback."""

from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass(frozen=True)
class FramePose:
    """One camera-chain pose saved at a dataset packet timestamp."""

    timestamp_ns: int
    theta_y_rad: float
    theta_p_rad: float


def interpolate_frame_pose(
    poses: Sequence[FramePose],
    timestamp_ns: int,
) -> Optional[FramePose]:
    """Linearly interpolate the camera-chain pose at one playback time.

    Dataset ``theta_y`` is deliberately stored unwrapped, so ordinary linear
    interpolation also works when the rig crosses a multiple of 2*pi. Frames
    are only defined over the packet time range: returning ``None`` outside it
    prevents the visualization from suggesting that the rig was stationary
    before or after the analysed steady-motion interval.
    """
    if not poses:
        return None
    if timestamp_ns < poses[0].timestamp_ns or timestamp_ns > poses[-1].timestamp_ns:
        return None

    # A short linear search would work, but binary search keeps every /clock
    # callback inexpensive even for a long recording containing many packets.
    lower = 0
    upper = len(poses)
    while lower < upper:
        middle = (lower + upper) // 2
        if poses[middle].timestamp_ns <= timestamp_ns:
            lower = middle + 1
        else:
            upper = middle
    right_index = lower

    if right_index == 0:
        return poses[0]
    if right_index == len(poses):
        return poses[-1]

    left_pose = poses[right_index - 1]
    right_pose = poses[right_index]
    interval_ns = right_pose.timestamp_ns - left_pose.timestamp_ns
    if interval_ns <= 0:
        raise ValueError("Frame-pose timestamps must be strictly increasing.")

    fraction = (timestamp_ns - left_pose.timestamp_ns) / interval_ns
    return FramePose(
        timestamp_ns=timestamp_ns,
        theta_y_rad=(
            left_pose.theta_y_rad
            + fraction * (right_pose.theta_y_rad - left_pose.theta_y_rad)
        ),
        theta_p_rad=(
            left_pose.theta_p_rad
            + fraction * (right_pose.theta_p_rad - left_pose.theta_p_rad)
        ),
    )
