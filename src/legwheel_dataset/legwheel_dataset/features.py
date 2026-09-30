"""Plane-slope and spectral features for Stage 2 extraction."""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class PlaneFit:
    """A robust plane and its quality measurements."""

    normal: FloatArray
    offset: float
    inlier_mask: NDArray[np.bool_]
    rmse_m: float


def _refine_plane(points: FloatArray) -> tuple[FloatArray, float]:
    centroid = np.mean(points, axis=0)
    _, _, right_vectors = np.linalg.svd(points - centroid, full_matrices=False)
    normal = right_vectors[-1]
    normal /= np.linalg.norm(normal)
    if normal[2] < 0.0:
        normal = -normal
    offset = -float(np.dot(normal, centroid))
    return normal, offset


def fit_plane_ransac(
    points: FloatArray,
    iterations: int,
    threshold_m: float,
    minimum_points: int,
) -> PlaneFit:
    """Fit a deterministic RANSAC plane and refine it on all inliers."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Plane points must have shape (N, 3).")
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    if points.shape[0] < minimum_points:
        raise ValueError(
            f"Plane needs at least {minimum_points} valid points; "
            f"received {points.shape[0]}."
        )

    rng = np.random.default_rng(0)
    best_inliers = None
    best_error = float("inf")
    for _ in range(iterations):
        sample = points[rng.choice(points.shape[0], size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        magnitude = np.linalg.norm(normal)
        if magnitude < 1e-12:
            continue
        normal /= magnitude
        offset = -float(np.dot(normal, sample[0]))
        distances = np.abs(points @ normal + offset)
        inliers = distances <= threshold_m
        inlier_count = int(np.count_nonzero(inliers))
        if inlier_count < 3:
            continue
        error = float(np.mean(distances[inliers] ** 2))
        if (
            best_inliers is None
            or inlier_count > int(np.count_nonzero(best_inliers))
            or (
                inlier_count == int(np.count_nonzero(best_inliers))
                and error < best_error
            )
        ):
            best_inliers = inliers
            best_error = error

    if best_inliers is None or np.count_nonzero(best_inliers) < minimum_points:
        raise ValueError("RANSAC could not find a plane with enough inliers.")
    normal, offset = _refine_plane(points[best_inliers])
    residuals = points[best_inliers] @ normal + offset
    rmse_m = float(np.sqrt(np.mean(residuals**2)))
    return PlaneFit(
        normal=normal,
        offset=offset,
        inlier_mask=best_inliers,
        rmse_m=rmse_m,
    )


def slope_from_plane(
    normal: FloatArray,
    midpoint_angle_rad: float,
    travel_direction: float,
) -> tuple[float, float]:
    """Return signed along-track and cross-track slopes in radians."""
    normal = np.asarray(normal, dtype=float)
    direction = 1.0 if travel_direction >= 0.0 else -1.0
    travel = direction * np.array(
        [
            -math.sin(midpoint_angle_rad),
            math.cos(midpoint_angle_rad),
            0.0,
        ]
    )
    cross_track = np.array([-travel[1], travel[0], 0.0])
    up_component = float(normal[2])
    along_slope = math.atan2(-float(np.dot(normal, travel)), up_component)
    cross_slope = math.atan2(
        -float(np.dot(normal, cross_track)),
        up_component,
    )
    return along_slope, cross_slope


def calculate_spectra(
    telemetry: FloatArray,
    valid: NDArray[np.bool_],
    rate_hz: float,
    minimum_valid_fraction: float,
) -> tuple[FloatArray, FloatArray, FloatArray]:
    """Calculate Hann-windowed one-sided power spectra per telemetry channel."""
    telemetry = np.asarray(telemetry, dtype=float)
    valid = np.asarray(valid, dtype=bool) & np.isfinite(telemetry)
    if telemetry.ndim != 2 or valid.shape != telemetry.shape:
        raise ValueError("Telemetry and validity mask must have equal 2-D shapes.")

    sample_count, channel_count = telemetry.shape
    frequencies_hz = np.fft.rfftfreq(sample_count, d=1.0 / rate_hz)
    power = np.full((frequencies_hz.size, channel_count), np.nan)
    dominant_frequency_hz = np.full(channel_count, np.nan)
    sample_positions = np.arange(sample_count, dtype=float)
    window = np.hanning(sample_count)
    window_energy = float(np.sum(window**2))

    for channel in range(channel_count):
        channel_valid = valid[:, channel]
        if np.mean(channel_valid) < minimum_valid_fraction:
            continue
        values = telemetry[:, channel]
        filled = np.interp(
            sample_positions,
            sample_positions[channel_valid],
            values[channel_valid],
        )
        slope, intercept = np.polyfit(sample_positions, filled, 1)
        detrended = filled - (slope * sample_positions + intercept)
        spectrum = np.fft.rfft(detrended * window)
        channel_power = np.abs(spectrum) ** 2 / (rate_hz * window_energy)
        if channel_power.size > 2:
            channel_power[1:-1] *= 2.0
        power[:, channel] = channel_power
        if channel_power.size > 1:
            dominant_index = 1 + int(np.argmax(channel_power[1:]))
            dominant_frequency_hz[channel] = frequencies_hz[dominant_index]
    return frequencies_hz, power, dominant_frequency_hz
