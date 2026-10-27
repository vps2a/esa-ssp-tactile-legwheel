"""Camera calibration and projection helpers for raw camera images."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]


@dataclass(frozen=True)
class CameraCalibration:
    """The subset of sensor_msgs/CameraInfo used by the processor."""

    width: int
    height: int
    K: FloatArray
    D: FloatArray
    distortion_model: str

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Camera calibration dimensions must be positive.")
        if self.K.shape != (3, 3) or not np.all(np.isfinite(self.K)):
            raise ValueError("Camera calibration K must be a finite 3x3 matrix.")
        if self.K[0, 0] <= 0.0 or self.K[1, 1] <= 0.0:
            raise ValueError("Camera calibration focal lengths must be positive.")
        if self.D.ndim != 1 or not np.all(np.isfinite(self.D)):
            raise ValueError("Camera distortion coefficients must be finite.")

    @classmethod
    def from_message(cls, message) -> "CameraCalibration":
        """Convert a ROS-like CameraInfo message without retaining ROS types."""
        return cls(
            width=int(message.width),
            height=int(message.height),
            K=np.asarray(message.k, dtype=float).reshape(3, 3),
            D=np.asarray(message.d, dtype=float),
            distortion_model=str(message.distortion_model),
        )

    def to_dict(self) -> dict:
        """Return JSON-serializable calibration values."""
        return {
            "width": self.width,
            "height": self.height,
            "K": self.K.tolist(),
            "D": self.D.tolist(),
            "distortion_model": self.distortion_model,
        }

    @classmethod
    def from_dict(cls, values: dict) -> "CameraCalibration":
        """Restore calibration values written into packet metadata."""
        return cls(
            width=int(values["width"]),
            height=int(values["height"]),
            K=np.asarray(values["K"], dtype=float),
            D=np.asarray(values["D"], dtype=float),
            distortion_model=str(values["distortion_model"]),
        )


def _distort_normalized(
    x: FloatArray,
    y: FloatArray,
    D: FloatArray,
    distortion_model: str,
) -> tuple[FloatArray, FloatArray]:
    """Apply ROS plumb-bob or rational-polynomial distortion."""
    if D.size == 0 or distortion_model == "":
        return x, y
    if distortion_model not in ("plumb_bob", "rational_polynomial"):
        raise ValueError(
            f"Unsupported camera distortion model: {distortion_model}"
        )

    coefficients = np.zeros(8, dtype=float)
    coefficients[: min(8, D.size)] = D[:8]
    k1, k2, p1, p2, k3, k4, k5, k6 = coefficients
    radius_squared = x * x + y * y
    numerator = 1.0 + k1 * radius_squared + k2 * radius_squared**2
    numerator += k3 * radius_squared**3
    denominator = 1.0 + k4 * radius_squared + k5 * radius_squared**2
    denominator += k6 * radius_squared**3
    radial = numerator / denominator
    x_distorted = (
        x * radial
        + 2.0 * p1 * x * y
        + p2 * (radius_squared + 2.0 * x * x)
    )
    y_distorted = (
        y * radial
        + p1 * (radius_squared + 2.0 * y * y)
        + 2.0 * p2 * x * y
    )
    return x_distorted, y_distorted


def project_camera_points(
    points_camera: FloatArray,
    calibration: CameraCalibration,
) -> tuple[FloatArray, BoolArray]:
    """Project camera-frame XYZ points into raw image pixel coordinates."""
    points_camera = np.asarray(points_camera, dtype=float)
    if points_camera.ndim != 2 or points_camera.shape[1] != 3:
        raise ValueError("points_camera must have shape (N, 3).")

    depth = points_camera[:, 2]
    in_front = depth > 0.0
    safe_depth = np.where(in_front, depth, 1.0)
    x = points_camera[:, 0] / safe_depth
    y = points_camera[:, 1] / safe_depth
    x, y = _distort_normalized(
        x,
        y,
        calibration.D,
        calibration.distortion_model,
    )

    u = calibration.K[0, 0] * x + calibration.K[0, 1] * y
    u += calibration.K[0, 2]
    v = calibration.K[1, 0] * x + calibration.K[1, 1] * y
    v += calibration.K[1, 2]
    pixels = np.column_stack((u, v))
    inside = (
        in_front
        & np.isfinite(u)
        & np.isfinite(v)
        & (u >= 0.0)
        & (u < calibration.width)
        & (v >= 0.0)
        & (v < calibration.height)
    )
    return pixels, inside


def undistort_pixels(
    pixels: FloatArray,
    calibration: CameraCalibration,
    iterations: int = 6,
) -> FloatArray:
    """Convert raw pixels to undistorted normalized image coordinates."""
    pixels = np.asarray(pixels, dtype=float)
    y_distorted = (pixels[:, 1] - calibration.K[1, 2]) / calibration.K[1, 1]
    x_distorted = (
        pixels[:, 0]
        - calibration.K[0, 2]
        - calibration.K[0, 1] * y_distorted
    ) / calibration.K[0, 0]

    x = x_distorted.copy()
    y = y_distorted.copy()
    for _ in range(iterations):
        estimate_x, estimate_y = _distort_normalized(
            x,
            y,
            calibration.D,
            calibration.distortion_model,
        )
        x += x_distorted - estimate_x
        y += y_distorted - estimate_y
    return np.column_stack((x, y))
