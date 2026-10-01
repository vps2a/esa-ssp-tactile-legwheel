"""Validated settings for deterministic post-processing."""

from dataclasses import asdict, dataclass
from datetime import datetime
import math
from numbers import Integral, Real
from pathlib import Path


CONFIG_METADATA_FIELDS = frozenset({"created_utc"})


@dataclass(frozen=True)
class ProcessingConfig:
    """Algorithm settings which are snapshotted with every derived dataset."""

    schema_version: str = "2.0"
    rgb_depth_max_delta_ms: float = 10.0
    spectral_window_ms: float = 1000.0
    telemetry_resample_hz: float = 100.0
    maximum_interpolation_gap_ms: float = 50.0
    maximum_packet_saving_frequency_hz: float = 10.0
    alpha_off_rad: float = 0.35
    alpha_sp_rad: float = 0.10
    # TODO: Replace this manual radius correction with a grouser-geometry and
    # experiment-based estimator.
    grouser_effective_height_m: float = 0.0
    minimum_target_torque_fraction: float = 0.90
    source_rate_tolerance_fraction: float = 0.05
    track_point_count: int = 360
    depth_scale_m: float = 0.001
    minimum_plane_points: int = 40
    plane_ransac_iterations: int = 200
    plane_inlier_threshold_m: float = 0.005
    minimum_spectral_valid_fraction: float = 0.8

    @classmethod
    def from_yaml(cls, path: Path | None) -> "ProcessingConfig":
        """Load an override file, or use documented defaults when omitted."""
        if path is None:
            config = cls()
        else:
            import yaml

            with path.open("r", encoding="utf-8") as file:
                values = yaml.safe_load(file)
            if not isinstance(values, dict):
                raise ValueError("Processing configuration must be a mapping.")
            known_fields = set(cls.__dataclass_fields__)
            unknown_fields = (
                set(values) - known_fields - CONFIG_METADATA_FIELDS
            )
            if unknown_fields:
                raise ValueError(
                    "Unknown processing configuration fields: "
                    + ", ".join(sorted(unknown_fields))
                )
            created_utc = values.get("created_utc")
            if created_utc is not None:
                _validate_created_utc(created_utc)
            algorithm_values = {
                name: value for name, value in values.items()
                if name in known_fields
            }
            config = cls(**algorithm_values)
        config.validate()
        return config

    def validate(self) -> None:
        """Reject invalid values before opening a potentially large bag."""
        if self.schema_version != "2.0":
            raise ValueError(
                f"Unsupported processing schema_version: {self.schema_version}"
            )
        integer_values = {
            "track_point_count": self.track_point_count,
            "minimum_plane_points": self.minimum_plane_points,
            "plane_ransac_iterations": self.plane_ransac_iterations,
        }
        for name, value in integer_values.items():
            if isinstance(value, bool) or not isinstance(value, Integral):
                raise ValueError(f"{name} must be a whole number.")
        positive_values = {
            "rgb_depth_max_delta_ms": self.rgb_depth_max_delta_ms,
            "spectral_window_ms": self.spectral_window_ms,
            "telemetry_resample_hz": self.telemetry_resample_hz,
            "maximum_interpolation_gap_ms": self.maximum_interpolation_gap_ms,
            "maximum_packet_saving_frequency_hz": (
                self.maximum_packet_saving_frequency_hz
            ),
            "alpha_off_rad": self.alpha_off_rad,
            "alpha_sp_rad": self.alpha_sp_rad,
            "depth_scale_m": self.depth_scale_m,
            "plane_inlier_threshold_m": self.plane_inlier_threshold_m,
        }
        for name, value in positive_values.items():
            if not _is_finite_number(value) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero.")
        if not _is_finite_number(self.grouser_effective_height_m):
            raise ValueError("grouser_effective_height_m must be finite.")
        if self.grouser_effective_height_m < 0.0:
            raise ValueError("grouser_effective_height_m must not be negative.")
        if self.alpha_sp_rad >= 2.0 * math.pi:
            raise ValueError("alpha_sp_rad must be smaller than one revolution.")
        if (
            not _is_finite_number(self.minimum_target_torque_fraction)
            or not 0.0 < self.minimum_target_torque_fraction <= 1.0
        ):
            raise ValueError(
                "minimum_target_torque_fraction must be in (0, 1]."
            )
        if (
            not _is_finite_number(self.source_rate_tolerance_fraction)
            or not 0.0 <= self.source_rate_tolerance_fraction <= 0.5
        ):
            raise ValueError(
                "source_rate_tolerance_fraction must be in [0, 0.5]."
            )
        if self.track_point_count < 16:
            raise ValueError("track_point_count must be at least 16.")
        if self.minimum_plane_points < 3:
            raise ValueError("minimum_plane_points must be at least 3.")
        if self.plane_ransac_iterations < 1:
            raise ValueError("plane_ransac_iterations must be positive.")
        if (
            not _is_finite_number(self.minimum_spectral_valid_fraction)
            or not 0.0 < self.minimum_spectral_valid_fraction <= 1.0
        ):
            raise ValueError(
                "minimum_spectral_valid_fraction must be in (0, 1]."
            )

    def to_dict(self) -> dict:
        """Return only algorithm values, excluding file metadata."""
        return asdict(self)


def _validate_created_utc(value) -> None:
    """Require an unambiguous, timezone-aware ISO 8601 creation time."""
    if not isinstance(value, str):
        raise ValueError("created_utc must be a quoted ISO 8601 string.")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError("created_utc must be a valid ISO 8601 timestamp.") from error
    if parsed.tzinfo is None:
        raise ValueError("created_utc must include a timezone offset.")


def _is_finite_number(value) -> bool:
    """Return true only for real, non-Boolean, finite numeric values."""
    return (
        not isinstance(value, bool)
        and isinstance(value, Real)
        and math.isfinite(float(value))
    )
