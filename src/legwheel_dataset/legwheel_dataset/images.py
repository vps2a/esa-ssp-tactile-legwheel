"""ROS image decoding, polygon masking, and depth point reconstruction."""

import numpy as np
from numpy.typing import NDArray

from legwheel_kinematics.projection import (
    CameraCalibration,
    undistort_pixels,
)


FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]


_ENCODINGS = {
    "rgb8": (np.uint8, 3),
    "bgr8": (np.uint8, 3),
    "rgba8": (np.uint8, 4),
    "bgra8": (np.uint8, 4),
    "mono8": (np.uint8, 1),
    "8UC1": (np.uint8, 1),
    "mono16": (np.uint16, 1),
    "16UC1": (np.uint16, 1),
    "32FC1": (np.float32, 1),
}


def decode_image(message) -> NDArray:
    """Convert a common sensor_msgs/Image encoding into a NumPy array."""
    if message.encoding not in _ENCODINGS:
        raise ValueError(f"Unsupported image encoding: {message.encoding}")
    native_dtype, channels = _ENCODINGS[message.encoding]
    dtype = np.dtype(native_dtype)
    if dtype.itemsize > 1:
        byte_order = ">" if bool(message.is_bigendian) else "<"
        dtype = dtype.newbyteorder(byte_order)

    width = int(message.width)
    height = int(message.height)
    row_bytes = width * channels * dtype.itemsize
    step = int(message.step)
    if step < row_bytes:
        raise ValueError("Image step is smaller than one decoded row.")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size != step * height:
        raise ValueError("Image data length does not match height and step.")

    rows = raw.reshape(height, step)[:, :row_bytes]
    image = rows.reshape(-1).view(dtype).reshape(height, width, channels)
    # Convert non-native byte order before numerical processing or .npy output.
    image = image.astype(native_dtype, copy=False)
    if channels == 1:
        return image[:, :, 0]
    return image


def polygon_mask(
    height: int,
    width: int,
    polygon_pixels: FloatArray,
) -> BoolArray:
    """Rasterize a pixel polygon with a dependency-free ray-casting test."""
    polygon = np.asarray(polygon_pixels, dtype=float)
    mask = np.zeros((height, width), dtype=bool)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or polygon.shape[0] < 3:
        return mask

    minimum = np.floor(np.min(polygon, axis=0)).astype(int)
    maximum = np.ceil(np.max(polygon, axis=0)).astype(int)
    min_u = max(0, int(minimum[0]))
    max_u = min(width - 1, int(maximum[0]))
    min_v = max(0, int(minimum[1]))
    max_v = min(height - 1, int(maximum[1]))
    if min_u > max_u or min_v > max_v:
        return mask

    u_grid, v_grid = np.meshgrid(
        np.arange(min_u, max_u + 1, dtype=float) + 0.5,
        np.arange(min_v, max_v + 1, dtype=float) + 0.5,
    )
    inside = np.zeros(u_grid.shape, dtype=bool)
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        crosses_row = (y1 > v_grid) != (y2 > v_grid)
        denominator = y2 - y1
        if abs(denominator) < 1e-12:
            previous = current
            continue
        intersection_u = x1 + (v_grid - y1) * (x2 - x1) / denominator
        inside ^= crosses_row & (u_grid < intersection_u)
        previous = current
    mask[min_v : max_v + 1, min_u : max_u + 1] = inside
    return mask


def masked_crop(image: NDArray, mask: BoolArray) -> tuple[NDArray, BoolArray]:
    """Return the smallest masked image crop and its equally sized mask."""
    rows, columns = np.nonzero(mask)
    if rows.size == 0:
        raise ValueError("The track polygon contains no image pixels.")
    row_slice = slice(int(rows.min()), int(rows.max()) + 1)
    column_slice = slice(int(columns.min()), int(columns.max()) + 1)
    crop = np.array(image[row_slice, column_slice], copy=True)
    crop_mask = mask[row_slice, column_slice]
    if crop.ndim == 3:
        crop[~crop_mask, :] = 0
    else:
        crop[~crop_mask] = 0
    return crop, crop_mask


def depth_points_in_base(
    depth_image: NDArray,
    mask: BoolArray,
    calibration: CameraCalibration,
    depth_scale_m: float,
    T_cam: FloatArray,
) -> tuple[FloatArray, float]:
    """Map valid masked depth pixels into the rig base coordinate frame."""
    if depth_image.ndim != 2:
        raise ValueError("Depth image must contain one scalar channel.")
    if depth_image.shape != mask.shape:
        raise ValueError("Depth image and ROI mask dimensions differ.")

    if np.issubdtype(depth_image.dtype, np.floating):
        depth_m = depth_image.astype(float)
    else:
        depth_m = depth_image.astype(float) * depth_scale_m
    valid_depth = mask & np.isfinite(depth_m) & (depth_m > 0.0)
    valid_fraction = float(np.count_nonzero(valid_depth)) / max(
        1,
        int(np.count_nonzero(mask)),
    )
    rows, columns = np.nonzero(valid_depth)
    if rows.size == 0:
        return np.empty((0, 3), dtype=float), valid_fraction

    pixels = np.column_stack((columns.astype(float), rows.astype(float)))
    normalized = undistort_pixels(pixels, calibration)
    depths = depth_m[rows, columns]
    points_camera = np.column_stack(
        (
            normalized[:, 0] * depths,
            normalized[:, 1] * depths,
            depths,
        )
    )
    homogeneous = np.column_stack(
        (points_camera, np.ones(points_camera.shape[0]))
    )
    points_base = (np.asarray(T_cam, dtype=float) @ homogeneous.T).T[:, :3]
    return points_base, valid_fraction
