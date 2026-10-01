"""Load the physical values required by the kinematics calculator."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any


# These are measured installation constants rather than experiment settings.
# Their names deliberately match supporting_files/kinematics_calculator.ipynb.
H_B_M = 0.0761
CAMERA_OFFSET_FROM_BEAM_CENTRE_M = 0.04

# The camera optical axis in the verified notebook geometry points in the
# decreasing kinematic-theta direction. Forward recordings are therefore those
# whose converted motion direction equals this value.
CAMERA_FORWARD_KINEMATIC_DIRECTION = -1.0


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
    theta_y_kinematic_sign: float

    def theta_y_to_kinematic(self, theta_y_motion_rad: float) -> float:
        """Convert encoder-motion theta_y into the notebook/base convention."""
        return self.theta_y_kinematic_sign * float(theta_y_motion_rad)

    def direction_to_kinematic(self, motion_direction: float) -> float:
        """Convert a signed encoder-motion direction into the base convention."""
        direction = float(motion_direction)
        if direction not in (-1.0, 1.0):
            raise ValueError("motion_direction must be exactly -1 or 1.")
        return self.theta_y_kinematic_sign * direction

    def is_forward_motion(self, motion_direction: float) -> bool:
        """Return whether motion follows the direction faced by the camera."""
        return (
            self.direction_to_kinematic(motion_direction)
            == CAMERA_FORWARD_KINEMATIC_DIRECTION
        )

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
        encoder_setup = _mapping(
            electronics_hardware,
            "encoder_setup",
            "electronics_hardware.encoder_setup",
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
        # Keep this at -1 for the current LegWheel geometry. The central
        # encoder increases during forward motion, while the notebook/DH
        # theta_y convention increases in the opposite direction. This field
        # exists to make that hardware-to-geometry mapping explicit; change it
        # only if the encoder mounting or kinematic convention changes.
        theta_y_kinematic_sign = _finite_number(
            encoder_setup,
            "theta_y_kinematic_sign",
            "electronics_hardware.encoder_setup.theta_y_kinematic_sign",
        )
        if theta_y_kinematic_sign not in (-1.0, 1.0):
            raise ValueError(
                "electronics_hardware.encoder_setup.theta_y_kinematic_sign "
                "must be exactly -1 or 1."
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
            theta_y_kinematic_sign=theta_y_kinematic_sign,
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
