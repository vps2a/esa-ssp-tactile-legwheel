"""Numerical implementation of the verified notebook transformation chain."""

import math

import numpy as np
from numpy.typing import NDArray

from legwheel_kinematics.config import H_B_M, KinematicsConfig


FloatArray = NDArray[np.float64]


# These names correspond to Base, F1, F2, F3, F4 and Camera in
# supporting_files/kinematics_calculator.ipynb.
KINEMATIC_FRAME_IDS = (
    "legwheel_base",
    "theta_y_link",
    "theta_p_link",
    "beam_end_link",
    "camera_mount_link",
    "legwheel_camera_optical_frame",
)


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
    return -math.asin(arcsin_argument)


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


def transformation_matrices_from_config(
    config: KinematicsConfig,
    theta_p: float,
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray, FloatArray]:
    """Return T01 through T45 using one experiment configuration."""
    # TODO: theta_y is consciously fixed at zero while the processed track is
    # radially circular. When theta_y is measured, T01 must also be published
    # as a dynamic transform by the post-processing visualization node.
    theta_y = 0.0

    return individual_transformation_matrices(
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


def frame_coordinates(
    config: KinematicsConfig,
    theta_p: float,
) -> tuple[FloatArray, ...]:
    """Return Base, F1, F2, F3, F4 and Camera poses in base coordinates."""
    T01, T12, T23, T34, T45 = transformation_matrices_from_config(
        config,
        theta_p,
    )

    # Keep the notebook name here so the implementation can be checked against
    # its Frame_coords calculation line by line.
    Frame_coords = [np.eye(4, dtype=float)]
    cumulative_transform = np.eye(4, dtype=float)
    for matrix in (T01, T12, T23, T34, T45):
        cumulative_transform = cumulative_transform @ matrix
        Frame_coords.append(cumulative_transform.copy())
    return tuple(Frame_coords)


def rotation_matrix_to_quaternion(
    rotation: FloatArray,
) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized (x, y, z, w) tuple."""
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation must be a finite 3x3 matrix.")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8):
        raise ValueError("rotation must be orthonormal.")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-8):
        raise ValueError("rotation must have determinant +1.")

    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale

    quaternion = np.asarray((x, y, z, w), dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    # q and -q describe the same orientation. A non-negative w makes the
    # result deterministic for tests and recorded visualization streams.
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return tuple(float(value) for value in quaternion)


def forward_kinematics(
    config: KinematicsConfig,
    theta_p: float,
) -> FloatArray:
    """Return T_cam, the camera-frame pose in the rig base frame."""
    return frame_coordinates(config, theta_p)[-1]
