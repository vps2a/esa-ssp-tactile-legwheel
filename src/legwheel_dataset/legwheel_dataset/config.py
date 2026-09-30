"""Validated settings for deterministic post-processing."""

from dataclasses import asdict, dataclass
import math
from pathlib import Path


@dataclass(frozen=True)
class ProcessingConfig:
    """Algorithm settings which are snapshotted with every derived dataset."""

    schema_version: str = "1.0"
    rgb_depth_max_delta_ms: float = 10.0
    local_window_ms: float = 100.0
    spectral_window_ms: float = 1000.0
    telemetry_resample_hz: float = 100.0
    maximum_interpolation_gap_ms: float = 50.0
    track_point_count: int = 360
    depth_scale_m: float = 0.001
    minimum_plane_points: int = 100
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
            unknown_fields = set(values) - known_fields
            if unknown_fields:
                raise ValueError(
                    "Unknown processing configuration fields: "
                    + ", ".join(sorted(unknown_fields))
                )
            config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        """Reject invalid values before opening a potentially large bag."""
        if self.schema_version != "1.0":
            raise ValueError(
                f"Unsupported processing schema_version: {self.schema_version}"
            )
        positive_values = {
            "rgb_depth_max_delta_ms": self.rgb_depth_max_delta_ms,
            "local_window_ms": self.local_window_ms,
            "spectral_window_ms": self.spectral_window_ms,
            "telemetry_resample_hz": self.telemetry_resample_hz,
            "maximum_interpolation_gap_ms": self.maximum_interpolation_gap_ms,
            "depth_scale_m": self.depth_scale_m,
            "plane_inlier_threshold_m": self.plane_inlier_threshold_m,
        }
        for name, value in positive_values.items():
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero.")
        if self.track_point_count < 16:
            raise ValueError("track_point_count must be at least 16.")
        if self.minimum_plane_points < 3:
            raise ValueError("minimum_plane_points must be at least 3.")
        if self.plane_ransac_iterations < 1:
            raise ValueError("plane_ransac_iterations must be positive.")
        if not 0.0 < self.minimum_spectral_valid_fraction <= 1.0:
            raise ValueError(
                "minimum_spectral_valid_fraction must be in (0, 1]."
            )
        if self.spectral_window_ms < self.local_window_ms:
            raise ValueError(
                "spectral_window_ms must not be shorter than local_window_ms."
            )

    def to_dict(self) -> dict:
        """Return JSON/YAML-safe configuration values."""
        return asdict(self)
