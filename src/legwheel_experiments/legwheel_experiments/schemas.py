"""Configuration models and validation for legwheel experiments.

The CLI, recorder, and ROS node all use these functions so that a configuration
has one definition of "valid" throughout the experiment lifecycle.
"""

from dataclasses import dataclass
from datetime import date, datetime
import math
from numbers import Real
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class ExperimentConfig:
    """Validated, experiment-wide values loaded from the YAML configuration."""

    schema_version: str
    experiment_id: str
    datetime_of_creation: str
    rig_config: dict[str, Any]
    leg_config: dict[str, Any]
    electronics_hardware: dict[str, Any]
    wheel_config: dict[str, Any]
    environment: dict[str, Any]


@dataclass(frozen=True)
class RunConfig:
    """Validated values selected by the operator for one experiment run."""

    run_length_loops: float
    leg_parameters: dict[str, float]
    wheel_parameters: dict[str, float]


# Top-level scalar values and YAML mappings must be checked differently.
REQUIRED_EXPERIMENT_FIELDS = (
    "schema_version",
    "experiment_id",
    "datetime_of_creation",
    "rig_config",
    "leg_config",
    "electronics_hardware",
    "wheel_config",
    "environment",
)

REQUIRED_EXPERIMENT_MAPPINGS = (
    "rig_config",
    "leg_config",
    "electronics_hardware",
    "wheel_config",
    "environment",
)


def require_mapping(parent: dict[str, Any], key: str, field_name: str) -> dict[str, Any]:
    """Return a required nested mapping, with an error naming its full path."""
    value = parent.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a YAML mapping.")
    return value


def require_nonempty_string(value: Any, field_name: str) -> str:
    """Require a non-empty text value for identifiers and version fields."""
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty string.")
    return value


def require_number(
    value: Any,
    field_name: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float:
    """Require a finite numeric value within the inclusive safety bounds."""
    # bool is a subclass of int in Python, but is never a valid hardware value.
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{field_name} must be a finite number.")

    numeric_value = float(value)
    if not math.isfinite(numeric_value):
        raise ValueError(f"{field_name} must be finite.")
    if minimum is not None and numeric_value < minimum:
        raise ValueError(f"{field_name} must be at least {minimum}.")
    if maximum is not None and numeric_value > maximum:
        raise ValueError(f"{field_name} must be at most {maximum}.")

    return numeric_value


def require_positive_number(value: Any, field_name: str) -> float:
    """Require a finite number that is strictly greater than zero."""
    numeric_value = require_number(value, field_name)
    if numeric_value <= 0.0:
        raise ValueError(f"{field_name} must be greater than zero.")
    return numeric_value


def validate_experiment_structure(raw_config: Any) -> dict[str, Any]:
    """Check the YAML shape before accessing its fields to build a dataclass."""
    if not isinstance(raw_config, dict):
        raise ValueError("Experiment configuration must contain a YAML mapping.")

    for field in REQUIRED_EXPERIMENT_FIELDS:
        if field not in raw_config:
            raise ValueError(f"Missing required field: {field}")

    for field in REQUIRED_EXPERIMENT_MAPPINGS:
        if not isinstance(raw_config[field], dict):
            raise ValueError(f"Field '{field}' must be a YAML mapping.")

    return raw_config


def load_experiment_config(config_path: Path) -> ExperimentConfig:
    """Load and fully validate an experiment configuration YAML file."""
    try:
        with config_path.open("r", encoding="utf-8") as file:
            raw_config = yaml.safe_load(file)
    except yaml.YAMLError as error:
        # Present malformed YAML as a normal configuration-validation error to the CLI.
        raise ValueError(f"Could not parse experiment configuration: {error}") from error

    raw_config = validate_experiment_structure(raw_config)
    creation_time = raw_config["datetime_of_creation"]
    # PyYAML may parse an unquoted ISO date or timestamp into a date object.
    if isinstance(creation_time, (date, datetime)):
        creation_time = creation_time.isoformat()

    config = ExperimentConfig(
        schema_version=raw_config["schema_version"],
        experiment_id=raw_config["experiment_id"],
        datetime_of_creation=creation_time,
        rig_config=raw_config["rig_config"],
        leg_config=raw_config["leg_config"],
        electronics_hardware=raw_config["electronics_hardware"],
        wheel_config=raw_config["wheel_config"],
        environment=raw_config["environment"],
    )
    validate_experiment_config(config)
    return config


def validate_experiment_config(config: ExperimentConfig) -> None:
    """Validate experiment values used by the experiment runner node."""
    # These checks also protect callers that construct ExperimentConfig directly.
    for field_name, section in (
        ("rig_config", config.rig_config),
        ("leg_config", config.leg_config),
        ("electronics_hardware", config.electronics_hardware),
        ("wheel_config", config.wheel_config),
        ("environment", config.environment),
    ):
        if not isinstance(section, dict):
            raise ValueError(f"{field_name} must be a YAML mapping.")

    require_nonempty_string(config.schema_version, "schema_version")
    require_nonempty_string(config.experiment_id, "experiment_id")
    creation_time = require_nonempty_string(
        config.datetime_of_creation,
        "datetime_of_creation",
    )
    try:
        if creation_time.endswith("Z"):
            creation_time = f"{creation_time[:-1]}+00:00"
        datetime.fromisoformat(creation_time)
    except ValueError as error:
        raise ValueError("datetime_of_creation must be ISO-8601 formatted.") from error

    beam_radius_m = require_positive_number(
        config.rig_config.get("beam_radius_m"),
        "rig_config.beam_radius_m",
    )
    require_positive_number(
        config.rig_config.get("beam_total_length_m"),
        "rig_config.beam_total_length_m",
    )
    require_positive_number(
        config.leg_config.get("hip_link_length_m"),
        "leg_config.hip_link_length_m",
    )
    require_positive_number(
        config.leg_config.get("calf_link_length_m"),
        "leg_config.calf_link_length_m",
    )
    require_positive_number(
        config.wheel_config.get("wheel_radius_mm"),
        "wheel_config.wheel_radius_mm",
    )
    wheel_width_mm = require_number(
        config.wheel_config.get("wheel_width_mm"),
        "wheel_config.wheel_width_mm",
        minimum=0.0,
    )

    # The wheel centreline must remain inside the beam radius used by _wheel_control.
    if wheel_width_mm >= 2.0 * beam_radius_m * 1000.0:
        raise ValueError(
            "wheel_config.wheel_width_mm must be smaller than the rig diameter."
        )

    encoder_setup = require_mapping(
        config.electronics_hardware,
        "encoder_setup",
        "electronics_hardware.encoder_setup",
    )
    require_positive_number(
        encoder_setup.get("ticks_per_legwheel_revolution"),
        "electronics_hardware.encoder_setup.ticks_per_legwheel_revolution",
    )


def make_run_config(raw_config: dict[str, Any]) -> RunConfig:
    """Build and validate a RunConfig from the raw CLI input mapping."""
    if not isinstance(raw_config, dict):
        raise ValueError("Run configuration must be a mapping.")

    try:
        leg_parameters = raw_config["leg_parameters"]
        wheel_parameters = raw_config["wheel_parameters"]
        config = RunConfig(
            run_length_loops=raw_config["run_length_loops"],
            leg_parameters=leg_parameters,
            wheel_parameters=wheel_parameters,
        )
    except KeyError as error:
        raise ValueError(
            f"Missing run configuration field: {error.args[0]}"
        ) from error

    if not isinstance(config.leg_parameters, dict):
        raise ValueError("leg_parameters must be a mapping.")
    if not isinstance(config.wheel_parameters, dict):
        raise ValueError("wheel_parameters must be a mapping.")

    validate_run_config(config)
    return config


def validate_run_config(config: RunConfig) -> None:
    """Validate all operator-selected values consumed during a run."""
    if not isinstance(config.leg_parameters, dict):
        raise ValueError("leg_parameters must be a mapping.")
    if not isinstance(config.wheel_parameters, dict):
        raise ValueError("wheel_parameters must be a mapping.")

    require_number(config.run_length_loops, "run_length_loops", minimum=0.0, maximum=2.0)
    require_number(
        config.leg_parameters.get("knee_stiffness_nm_per_rad"),
        "leg_parameters.knee_stiffness_nm_per_rad",
        minimum=0.0,
        maximum=50.0,
    )
    require_number(
        config.leg_parameters.get("knee_spring_zeroposition_rad"),
        "leg_parameters.knee_spring_zeroposition_rad",
        minimum=0.0,
        maximum=1.0,
    )
    require_number(
        config.leg_parameters.get("knee_damping_nms_per_rad"),
        "leg_parameters.knee_damping_nms_per_rad",
        minimum=0.0,
        maximum=5.0,
    )
    require_number(
        config.wheel_parameters.get("commanded_wheel_torque_nm"),
        "wheel_parameters.commanded_wheel_torque_nm",
        minimum=0.0,
        maximum=8.0,
    )
    require_number(
        config.wheel_parameters.get("wheel_torque_ramp_time_sec"),
        "wheel_parameters.wheel_torque_ramp_time_sec",
        minimum=0.05,
        maximum=5.0,
    )
