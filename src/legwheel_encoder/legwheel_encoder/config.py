"""Encoder settings shared by acquisition and offline reconstruction.

Published ticks have already been zeroed and direction-adjusted by the driver.
Offline consumers only subtract the recording origin and apply the calibration;
applying ``invert_direction`` again would reverse an existing recording.
"""

from dataclasses import dataclass
import math
from numbers import Real
from pathlib import Path

import yaml


TICKS_TOPIC = "/rotation_encoder/ticks"
CONFIG_TOPIC = "/rotation_encoder/config"
ENCODER_POSITION_VERSION = 1


def encoder_setup(experiment: dict) -> dict:
    """Find the one encoder section, with a useful error for unmigrated files."""
    try:
        setup = experiment["electronics_hardware"]["encoder_setup"]
    except (KeyError, TypeError) as error:
        raise ValueError("electronics_hardware.encoder_setup is required.") from error
    if not isinstance(setup, dict):
        raise ValueError("electronics_hardware.encoder_setup must be a mapping.")
    return setup


def read_experiment(path: Path) -> dict:
    """Load an experiment file without ROS dependencies."""
    with Path(path).open(encoding="utf-8") as stream:
        experiment = yaml.safe_load(stream)
    if not isinstance(experiment, dict):
        raise ValueError("Experiment configuration must be a YAML mapping.")
    return experiment


def _positive_number(setup: dict, name: str) -> float:
    value = setup.get(name)
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError(
            f"electronics_hardware.encoder_setup.{name} must be finite and positive."
        )
    return float(value)


@dataclass(frozen=True)
class EncoderCalibration:
    """The scale used for signed counts from /rotation_encoder/ticks."""

    ticks_per_legwheel_revolution: float

    @classmethod
    def from_experiment_dict(cls, experiment: dict) -> "EncoderCalibration":
        return cls(
            _positive_number(encoder_setup(experiment), "ticks_per_legwheel_revolution")
        )

    @property
    def radians_per_tick(self) -> float:
        return 2.0 * math.pi / self.ticks_per_legwheel_revolution

    def motion_angle(self, ticks, origin_ticks=0):
        """Convert a scalar or NumPy array, preserving complete revolutions."""
        return (ticks - origin_ticks) * self.radians_per_tick


@dataclass(frozen=True)
class EncoderConfig:
    """All driver settings, required explicitly in the experiment config."""

    port: str
    baudrate: int
    ticks_per_legwheel_revolution: float
    publish_rate_hz: float
    invert_direction: bool
    zero_on_start: bool
    theta_y_kinematic_sign: float

    @classmethod
    def from_setup(cls, setup: dict) -> "EncoderConfig":
        if not isinstance(setup, dict):
            raise ValueError("electronics_hardware.encoder_setup must be a mapping.")
        port = setup.get("port")
        if not isinstance(port, str) or not port.strip():
            raise ValueError(
                "electronics_hardware.encoder_setup.port must be a nonempty string."
            )
        baudrate = setup.get("baudrate")
        if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError(
                "electronics_hardware.encoder_setup.baudrate must be a positive integer."
            )
        for name in ("invert_direction", "zero_on_start"):
            if not isinstance(setup.get(name), bool):
                raise ValueError(
                    f"electronics_hardware.encoder_setup.{name} must be a boolean."
                )
        sign = setup.get("theta_y_kinematic_sign")
        if isinstance(sign, bool) or not isinstance(sign, Real) or sign not in (-1, 1):
            raise ValueError(
                "electronics_hardware.encoder_setup.theta_y_kinematic_sign must be exactly -1 or 1."
            )
        return cls(
            port=port,
            baudrate=baudrate,
            ticks_per_legwheel_revolution=_positive_number(
                setup, "ticks_per_legwheel_revolution"
            ),
            publish_rate_hz=_positive_number(setup, "publish_rate_hz"),
            invert_direction=setup["invert_direction"],
            zero_on_start=setup["zero_on_start"],
            theta_y_kinematic_sign=float(sign),
        )

    @classmethod
    def from_yaml(cls, path: Path) -> "EncoderConfig":
        return cls.from_setup(encoder_setup(read_experiment(path)))

    @property
    def calibration(self) -> EncoderCalibration:
        return EncoderCalibration(self.ticks_per_legwheel_revolution)


class DriverTickCounter:
    """Apply startup zeroing and hardware direction once, before publication."""

    def __init__(self, *, zero_on_start: bool, invert_direction: bool):
        self.zero_on_start = zero_on_start
        self.direction = -1 if invert_direction else 1
        self.origin_ticks: int | None = None

    def update(self, hardware_ticks: int) -> int:
        if self.origin_ticks is None:
            self.origin_ticks = hardware_ticks if self.zero_on_start else 0
        return self.direction * (hardware_ticks - self.origin_ticks)
