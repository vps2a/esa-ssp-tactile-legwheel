"""Generate and project the circular wheel track used by the notebook."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.projection import (
    CameraCalibration,
    project_camera_points,
)
from legwheel_kinematics.transforms import forward_kinematics


FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]


@dataclass(frozen=True)
class TrackProjection:
    """Projected inner/outer track boundaries and the visible patch polygon."""

    angles: FloatArray
    inner_pixels: FloatArray
    outer_pixels: FloatArray
    valid: BoolArray
    visible_indices: NDArray[np.int64]
    polygon_pixels: FloatArray
    midpoint_angle_rad: float


@dataclass(frozen=True)
class TrackPoints:
    """Inner and outer circular track samples in legwheel_base coordinates."""

    angles: FloatArray
    inner_track_points: FloatArray
    outer_track_points: FloatArray


def generate_track_points(
    config: KinematicsConfig,
    point_count: int = 360,
) -> TrackPoints:
    """Generate the same 3D circular track points as the notebook."""
    if point_count < 16:
        raise ValueError("point_count must be at least 16.")

    Beam_radius = config.Beam_radius
    Wheel_width = config.Wheel_width
    inner_track_radius = Beam_radius - Wheel_width / 2.0
    outer_track_radius = Beam_radius + Wheel_width / 2.0
    if inner_track_radius <= 0.0:
        raise ValueError("Wheel width leaves no positive inner track radius.")

    angles = np.linspace(-np.pi, np.pi, point_count, endpoint=False)
    inner_track_points = np.column_stack(
        (
            inner_track_radius * np.cos(angles),
            inner_track_radius * np.sin(angles),
            np.zeros(point_count),
        )
    )
    outer_track_points = np.column_stack(
        (
            outer_track_radius * np.cos(angles),
            outer_track_radius * np.sin(angles),
            np.zeros(point_count),
        )
    )
    return TrackPoints(
        angles=angles,
        inner_track_points=inner_track_points,
        outer_track_points=outer_track_points,
    )


def _longest_circular_run(mask: BoolArray) -> NDArray[np.int64]:
    """Return indexes of the longest true run in a circular Boolean array."""
    size = int(mask.size)
    if size == 0 or not np.any(mask):
        return np.empty(0, dtype=np.int64)
    if np.all(mask):
        return np.arange(size, dtype=np.int64)

    doubled = np.concatenate((mask, mask))
    best_start = 0
    best_length = 0
    current_start = 0
    current_length = 0
    for index, enabled in enumerate(doubled):
        if enabled:
            if current_length == 0:
                current_start = index
            current_length += 1
            if current_length > best_length and current_length <= size:
                best_start = current_start
                best_length = current_length
        else:
            current_length = 0
    return np.asarray(
        [(best_start + offset) % size for offset in range(best_length)],
        dtype=np.int64,
    )


def project_track(
    config: KinematicsConfig,
    theta_p: float,
    calibration: CameraCalibration,
    point_count: int = 360,
) -> TrackProjection:
    """Project both circular track boundaries and form the visible ROI."""
    track_points = generate_track_points(config, point_count)
    angles = track_points.angles
    inner_track_points = track_points.inner_track_points
    outer_track_points = track_points.outer_track_points

    T_cam = forward_kinematics(config, theta_p)
    T_cam_to_base = np.linalg.inv(T_cam)

    def to_camera(points: FloatArray) -> FloatArray:
        homogeneous = np.column_stack((points, np.ones(points.shape[0])))
        return (T_cam_to_base @ homogeneous.T).T[:, :3]

    inner_pixels, inner_valid = project_camera_points(
        to_camera(inner_track_points),
        calibration,
    )
    outer_pixels, outer_valid = project_camera_points(
        to_camera(outer_track_points),
        calibration,
    )
    valid = inner_valid & outer_valid
    visible_indices = _longest_circular_run(valid)

    if visible_indices.size:
        polygon_pixels = np.vstack(
            (
                inner_pixels[visible_indices],
                outer_pixels[visible_indices[::-1]],
            )
        )
        unwrapped_angles = np.unwrap(angles[visible_indices])
        midpoint_angle_rad = float(np.mean(unwrapped_angles))
        midpoint_angle_rad = float(
            (midpoint_angle_rad + np.pi) % (2.0 * np.pi) - np.pi
        )
    else:
        polygon_pixels = np.empty((0, 2), dtype=float)
        midpoint_angle_rad = float("nan")

    return TrackProjection(
        angles=angles,
        inner_pixels=inner_pixels,
        outer_pixels=outer_pixels,
        valid=valid,
        visible_indices=visible_indices,
        polygon_pixels=polygon_pixels,
        midpoint_angle_rad=midpoint_angle_rad,
    )
