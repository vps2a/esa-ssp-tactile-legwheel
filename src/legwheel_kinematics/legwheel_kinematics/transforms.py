"""Numerical implementation of the verified notebook transformation chain."""

import math

import numpy as np
from numpy.typing import NDArray

from legwheel_kinematics.config import H_B_M, KinematicsConfig


FloatArray = NDArray[np.float64]


def calculate_theta_p(
    knee_joint: float,
    wheel_radius_mm: float,
    hip_link_length_m: float,
    calf_link_length_m: float,
    pivot_height_m: float,
    h_b: float = H_B_M,
) -> float:
    """Calculate theta_p from the measured knee joint angle.

    The equation is the rig-specific relationship supplied for this mechanism.
    It is kept in one function so its physical convention can be tested without
    requiring ROS messages or a rosbag.
    """
    leg_length_squared = (
        hip_link_length_m**2
        + calf_link_length_m**2
        - 2.0
        * hip_link_length_m
        * calf_link_length_m
        * math.cos(knee_joint)
    )
    # Floating point round-off can make an exact zero slightly negative.
    if leg_length_squared < -1e-12:
        raise ValueError("The knee geometry produced a negative squared length.")
    leg_length = math.sqrt(max(0.0, leg_length_squared))

    fixed_vertical_offset_m = 0.085
    arcsin_argument = (
        wheel_radius_mm / 1000.0
        + fixed_vertical_offset_m
        + leg_length
        - h_b
        - pivot_height_m
    )
    if not -1.0 <= arcsin_argument <= 1.0:
        raise ValueError(
            "theta_p is undefined because its arcsin argument is outside "
            f"[-1, 1]: {arcsin_argument}"
        )
    return math.asin(arcsin_argument)


def theta_p_from_config(knee_joint: float, config: KinematicsConfig) -> float:
    """Calculate theta_p using the values loaded from an experiment file."""
    return calculate_theta_p(
        knee_joint=knee_joint,
        wheel_radius_mm=config.wheel_radius_mm,
        hip_link_length_m=config.hip_link_length_m,
        calf_link_length_m=config.calf_link_length_m,
        pivot_height_m=config.d1,
        h_b=config.h_b,
    )


def individual_transformation_matrices(
    d1: float,
    theta_y: float,
    theta_p: float,
    a_b: float,
    h_b: float,
    camera_beam_offset: float,
    camera_height: float,
    camera_offset_from_beam_centre: float,
    camera_angle_deg: float,
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray, FloatArray]:
    """Return T01, T12, T23, T34 and T45 from the notebook."""
    cos_y = math.cos(theta_y)
    sin_y = math.sin(theta_y)
    cos_p = math.cos(theta_p)
    sin_p = math.sin(theta_p)
    camera_angle_rad = math.radians(camera_angle_deg)
    cos_camera = math.cos(camera_angle_rad)
    sin_camera = math.sin(camera_angle_rad)

    T01 = np.array(
        [
            [cos_y, -sin_y, 0.0, 0.0],
            [sin_y, cos_y, 0.0, 0.0],
            [0.0, 0.0, 1.0, d1],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    T12 = np.array(
        [
            [cos_p, -sin_p, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [sin_p, cos_p, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    T23 = np.array(
        [
            [1.0, 0.0, 0.0, a_b],
            [0.0, 1.0, 0.0, h_b],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    T34 = np.array(
        [
            [1.0, 0.0, 0.0, camera_beam_offset],
            [0.0, 1.0, 0.0, camera_height],
            [0.0, 0.0, 1.0, camera_offset_from_beam_centre],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    T45 = np.array(
        [
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -cos_camera, -sin_camera, 0.0],
            [0.0, -sin_camera, cos_camera, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return T01, T12, T23, T34, T45


def forward_kinematics(
    config: KinematicsConfig,
    theta_p: float,
) -> FloatArray:
    """Return T_cam, the camera-frame pose in the rig base frame."""
    # TODO: theta_y is intentionally fixed at zero for now. The track is
    # radially circular, so rotating the complete local geometry does not alter
    # its camera projection. Restore the measured theta_y when the model starts
    # representing non-circular or world-fixed geometry.
    theta_y = 0.0

    matrices = individual_transformation_matrices(
        d1=config.d1,
        theta_y=theta_y,
        theta_p=theta_p,
        a_b=config.a_b,
        h_b=config.h_b,
        camera_beam_offset=config.camera_beam_offset,
        camera_height=config.camera_height,
        camera_offset_from_beam_centre=(
            config.camera_offset_from_beam_centre
        ),
        camera_angle_deg=config.camera_angle_deg,
    )
    T_cam = np.eye(4, dtype=float)
    for matrix in matrices:
        T_cam = T_cam @ matrix
    return T_cam
