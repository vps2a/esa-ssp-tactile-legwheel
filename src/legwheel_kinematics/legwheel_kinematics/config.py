"""Load the physical values required by the kinematics calculator."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any


# These are measured installation constants rather than experiment settings.
# Their names deliberately match supporting_files/kinematics_calculator.ipynb.
H_B_M = 0.0761
CAMERA_OFFSET_FROM_BEAM_CENTRE_M = 0.04


def _mapping(parent: dict[str, Any], key: str, path: str) -> dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping.")
    return value


def _finite_number(parent: dict[str, Any], key: str, path: str) -> float:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be a finite number.")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{path} must be a finite number.")
    return value


@dataclass(frozen=True)
class KinematicsConfig:
    """Numerical inputs named after the verified kinematics notebook."""

    d1: float
    a_b: float
    h_b: float
    camera_beam_offset: float
    camera_height: float
    camera_offset_from_beam_centre: float
    camera_angle_deg: float
    Beam_radius: float
    Wheel_width: float
    wheel_radius_mm: float
    hip_link_length_m: float
    calf_link_length_m: float

    @classmethod
    def from_experiment_dict(
        cls,
        experiment_config: dict[str, Any],
    ) -> "KinematicsConfig":
        """Create the notebook inputs from one experiment configuration."""
        rig_config = _mapping(
            experiment_config,
            "rig_config",
            "rig_config",
        )
        leg_config = _mapping(
            experiment_config,
            "leg_config",
            "leg_config",
        )
        electronics_hardware = _mapping(
            experiment_config,
            "electronics_hardware",
            "electronics_hardware",
        )
        camera = _mapping(
            electronics_hardware,
            "camera",
            "electronics_hardware.camera",
        )
        camera_config = _mapping(
            camera,
            "camera_config",
            "electronics_hardware.camera.camera_config",
        )
        wheel_config = _mapping(
            experiment_config,
            "wheel_config",
            "wheel_config",
        )

        d1 = _finite_number(rig_config, "pivot_height_m", "rig_config.pivot_height_m")
        a_b = _finite_number(rig_config, "beam_radius_m", "rig_config.beam_radius_m")
        camera_beam_offset = _finite_number(
            camera_config,
            "camera_beam_offset_m",
            "electronics_hardware.camera.camera_config.camera_beam_offset_m",
        )
        camera_height = _finite_number(
            camera_config,
            "camera_height_m",
            "electronics_hardware.camera.camera_config.camera_height_m",
        )
        camera_angle_deg = _finite_number(
            camera_config,
            "camera_front_angle_deg",
            "electronics_hardware.camera.camera_config.camera_front_angle_deg",
        )
        wheel_radius_mm = _finite_number(
            wheel_config,
            "wheel_radius_mm",
            "wheel_config.wheel_radius_mm",
        )
        wheel_width_mm = _finite_number(
            wheel_config,
            "wheel_width_mm",
            "wheel_config.wheel_width_mm",
        )
        hip_link_length_m = _finite_number(
            leg_config,
            "hip_link_length_m",
            "leg_config.hip_link_length_m",
        )
        calf_link_length_m = _finite_number(
            leg_config,
            "calf_link_length_m",
            "leg_config.calf_link_length_m",
        )

        positive_values = {
            "rig_config.pivot_height_m": d1,
            "rig_config.beam_radius_m": a_b,
            "wheel_config.wheel_radius_mm": wheel_radius_mm,
            "wheel_config.wheel_width_mm": wheel_width_mm,
            "leg_config.hip_link_length_m": hip_link_length_m,
            "leg_config.calf_link_length_m": calf_link_length_m,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be greater than zero.")

        return cls(
            d1=d1,
            a_b=a_b,
            h_b=H_B_M,
            camera_beam_offset=camera_beam_offset,
            camera_height=camera_height,
            camera_offset_from_beam_centre=(
                CAMERA_OFFSET_FROM_BEAM_CENTRE_M
            ),
            camera_angle_deg=camera_angle_deg,
            Beam_radius=a_b,
            Wheel_width=wheel_width_mm / 1000.0,
            wheel_radius_mm=wheel_radius_mm,
            hip_link_length_m=hip_link_length_m,
            calf_link_length_m=calf_link_length_m,
        )

    @classmethod
    def from_yaml(cls, config_path: Path) -> "KinematicsConfig":
        """Load an experiment YAML file and create its kinematics inputs."""
        import yaml

        with config_path.open("r", encoding="utf-8") as file:
            experiment_config = yaml.safe_load(file)
        if not isinstance(experiment_config, dict):
            raise ValueError("Experiment configuration must contain a mapping.")
        return cls.from_experiment_dict(experiment_config)
