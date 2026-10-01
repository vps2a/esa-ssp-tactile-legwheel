"""Timestamp pairing and one-dimensional interpolation helpers."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


IntArray = NDArray[np.int64]
FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class ImagePair:
    """Indexes and sensor times for one accepted RGB-depth pair."""

    rgb_index: int
    depth_index: int
    rgb_time_ns: int
    depth_time_ns: int

    @property
    def image_time_ns(self) -> int:
        return (self.rgb_time_ns + self.depth_time_ns) // 2

    @property
    def sync_error_ns(self) -> int:
        return self.depth_time_ns - self.rgb_time_ns


def pair_image_timestamps(
    rgb_times_ns: IntArray,
    depth_times_ns: IntArray,
    maximum_delta_ns: int,
) -> list[ImagePair]:
    """Pair monotonic image stamps one-to-one with nearest depth frames."""
    rgb_times_ns = np.asarray(rgb_times_ns, dtype=np.int64)
    depth_times_ns = np.asarray(depth_times_ns, dtype=np.int64)
    if maximum_delta_ns < 0:
        raise ValueError("maximum_delta_ns must not be negative.")
    if np.any(np.diff(rgb_times_ns) <= 0):
        raise ValueError("RGB timestamps must be strictly increasing.")
    if np.any(np.diff(depth_times_ns) <= 0):
        raise ValueError("Depth timestamps must be strictly increasing.")

    pairs = []
    depth_index = 0
    for rgb_index, rgb_time_ns in enumerate(rgb_times_ns):
        if depth_index >= depth_times_ns.size:
            break
        while (
            depth_index + 1 < depth_times_ns.size
            and abs(int(depth_times_ns[depth_index + 1]) - int(rgb_time_ns))
            < abs(int(depth_times_ns[depth_index]) - int(rgb_time_ns))
        ):
            depth_index += 1
        depth_time_ns = int(depth_times_ns[depth_index])
        if abs(depth_time_ns - int(rgb_time_ns)) <= maximum_delta_ns:
            pairs.append(
                ImagePair(
                    rgb_index=rgb_index,
                    depth_index=depth_index,
                    rgb_time_ns=int(rgb_time_ns),
                    depth_time_ns=depth_time_ns,
                )
            )
            depth_index += 1
    return pairs


def interpolate_scalar(
    times_ns: IntArray,
    values: FloatArray,
    target_time_ns: int,
) -> float:
    """Linearly interpolate a scalar within a monotonic timestamp series."""
    times_ns = np.asarray(times_ns, dtype=np.int64)
    values = np.asarray(values, dtype=float)
    if times_ns.size != values.size or times_ns.size < 2:
        raise ValueError("At least two timestamped scalar samples are required.")
    if np.any(np.diff(times_ns) <= 0):
        raise ValueError("Scalar timestamps must be strictly increasing.")
    if target_time_ns < times_ns[0] or target_time_ns > times_ns[-1]:
        raise ValueError("Target time lies outside the scalar sample range.")

    right = int(np.searchsorted(times_ns, target_time_ns, side="left"))
    if right < times_ns.size and times_ns[right] == target_time_ns:
        return float(values[right])
    left = right - 1
    fraction = (target_time_ns - int(times_ns[left])) / (
        int(times_ns[right]) - int(times_ns[left])
    )
    return float(values[left] + fraction * (values[right] - values[left]))


def resample_channels(
    source_times_ns: IntArray,
    source_values: FloatArray,
    centre_time_ns: int,
    window_ms: float,
    rate_hz: float,
    maximum_gap_ms: float,
) -> tuple[FloatArray, NDArray[np.bool_], IntArray]:
    """Resample telemetry onto a fixed relative-time grid.

    Each output cell has a validity mask. Interpolation across a source gap
    larger than maximum_gap_ms is intentionally left invalid.
    """
    source_times_ns = np.asarray(source_times_ns, dtype=np.int64)
    source_values = np.asarray(source_values, dtype=float)
    if source_values.ndim == 1:
        source_values = source_values[:, None]
    if source_values.shape[0] != source_times_ns.size:
        raise ValueError("Telemetry timestamps and values have different lengths.")
    if np.any(np.diff(source_times_ns) <= 0):
        raise ValueError("Telemetry timestamps must be strictly increasing.")

    sample_count = max(2, int(round(window_ms * rate_hz / 1000.0)) + 1)
    half_window_ns = int(round(window_ms * 1_000_000.0 / 2.0))
    relative_times_ns = np.rint(
        np.linspace(-half_window_ns, half_window_ns, sample_count)
    ).astype(np.int64)
    target_times_ns = relative_times_ns + int(centre_time_ns)
    output = np.full((sample_count, source_values.shape[1]), np.nan)
    valid = np.zeros(output.shape, dtype=bool)
    maximum_gap_ns = int(round(maximum_gap_ms * 1_000_000.0))

    for output_index, target_time_ns in enumerate(target_times_ns):
        right = int(np.searchsorted(source_times_ns, target_time_ns, side="left"))
        if right < source_times_ns.size and source_times_ns[right] == target_time_ns:
            output[output_index] = source_values[right]
            valid[output_index] = np.isfinite(source_values[right])
            continue
        if right == 0 or right == source_times_ns.size:
            continue
        left = right - 1
        gap_ns = int(source_times_ns[right]) - int(source_times_ns[left])
        if gap_ns > maximum_gap_ns:
            continue
        fraction = (target_time_ns - int(source_times_ns[left])) / gap_ns
        interpolated = source_values[left] + fraction * (
            source_values[right] - source_values[left]
        )
        output[output_index] = interpolated
        valid[output_index] = np.isfinite(interpolated)
    return output, valid, relative_times_ns
