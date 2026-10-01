"""Physical track-patch geometry and camera projection for offline packets."""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.projection import CameraCalibration, project_camera_points
from legwheel_kinematics.transforms import individual_transformation_matrices


FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]


@dataclass(frozen=True)
class PatchProjection:
    """One physical annular-sector patch and its complete image projection."""

    angles_rad: FloatArray
    inner_points_base_m: FloatArray
    outer_points_base_m: FloatArray
    inner_pixels: FloatArray
    outer_pixels: FloatArray
    polygon_pixels: FloatArray
    fully_visible: bool
    T_camera_in_base: FloatArray


def camera_pose_in_base(
    config: KinematicsConfig,
    theta_y: float,
    theta_p: float,
) -> FloatArray:
    """Return the verified notebook camera pose with processed theta_y applied."""
    matrices = individual_transformation_matrices(
        d1=config.d1,
        theta_y=theta_y,
        theta_p=theta_p,
        a_b=config.a_b,
        h_b=config.h_b,
        camera_beam_offset=config.camera_beam_offset,
        camera_height=config.camera_height,
        camera_offset_from_beam_centre=config.camera_offset_from_beam_centre,
        camera_angle_deg=config.camera_angle_deg,
    )
    transform = np.eye(4, dtype=float)
    for matrix in matrices:
        transform = transform @ matrix
    return transform


def _points_at_angles(radius_m: float, angles_rad: FloatArray) -> FloatArray:
    return np.column_stack(
        (
            radius_m * np.cos(angles_rad),
            radius_m * np.sin(angles_rad),
            np.zeros(angles_rad.size, dtype=float),
        )
    )


def _to_camera(points_base: FloatArray, T_camera_in_base: FloatArray) -> FloatArray:
    homogeneous = np.column_stack((points_base, np.ones(points_base.shape[0])))
    T_base_in_camera = np.linalg.inv(T_camera_in_base)
    return (T_base_in_camera @ homogeneous.T).T[:, :3]


def project_track_patch(
    config: KinematicsConfig,
    theta_y: float,
    theta_p: float,
    patch_centre_angle_rad: float,
    patch_spread_rad: float,
    calibration: CameraCalibration,
    track_point_count: int,
) -> PatchProjection:
    """Project an exact annular-sector patch into one raw camera image.

    ``track_point_count`` supplies the full-circle angular density.  The exact
    start and end angles are always inserted so mask width is not quantised by
    that density.
    """
    if track_point_count < 16:
        raise ValueError("track_point_count_must_be_at_least_16")
    if not 0.0 < patch_spread_rad < 2.0 * math.pi:
        raise ValueError("invalid_patch_angular_spread")

    inner_radius_m = config.Beam_radius - config.Wheel_width / 2.0
    outer_radius_m = config.Beam_radius + config.Wheel_width / 2.0
    if inner_radius_m <= 0.0:
        raise ValueError("nonpositive_inner_track_radius")

    full_circle_step = 2.0 * math.pi / track_point_count
    interval_count = max(1, int(math.ceil(patch_spread_rad / full_circle_step)))
    start_angle = patch_centre_angle_rad - patch_spread_rad / 2.0
    end_angle = patch_centre_angle_rad + patch_spread_rad / 2.0
    angles = np.linspace(start_angle, end_angle, interval_count + 1)

    inner_points = _points_at_angles(inner_radius_m, angles)
    outer_points = _points_at_angles(outer_radius_m, angles)
    T_camera = camera_pose_in_base(config, theta_y, theta_p)
    inner_pixels, inner_valid = project_camera_points(
        _to_camera(inner_points, T_camera),
        calibration,
    )
    outer_pixels, outer_valid = project_camera_points(
        _to_camera(outer_points, T_camera),
        calibration,
    )
    fully_visible = bool(np.all(inner_valid) and np.all(outer_valid))
    polygon = np.vstack((inner_pixels, outer_pixels[::-1]))
    return PatchProjection(
        angles_rad=angles,
        inner_points_base_m=inner_points,
        outer_points_base_m=outer_points,
        inner_pixels=inner_pixels,
        outer_pixels=outer_pixels,
        polygon_pixels=polygon,
        fully_visible=fully_visible,
        T_camera_in_base=T_camera,
    )


def project_full_track_boundaries(
    config: KinematicsConfig,
    theta_y: float,
    theta_p: float,
    calibration: CameraCalibration,
    track_point_count: int,
) -> tuple[FloatArray, BoolArray, FloatArray, BoolArray]:
    """Project the complete inner and outer boundaries for Foxglove overlays."""
    angles = np.linspace(-math.pi, math.pi, track_point_count, endpoint=False)
    inner_radius_m = config.Beam_radius - config.Wheel_width / 2.0
    outer_radius_m = config.Beam_radius + config.Wheel_width / 2.0
    T_camera = camera_pose_in_base(config, theta_y, theta_p)
    inner_pixels, inner_valid = project_camera_points(
        _to_camera(_points_at_angles(inner_radius_m, angles), T_camera),
        calibration,
    )
    outer_pixels, outer_valid = project_camera_points(
        _to_camera(_points_at_angles(outer_radius_m, angles), T_camera),
        calibration,
    )
    return inner_pixels, inner_valid, outer_pixels, outer_valid
