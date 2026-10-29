"""Run wheel settings and the exact package configuration used for defaults."""

from dataclasses import dataclass
from collections.abc import Sequence
import json
import math
from pathlib import Path
from typing import Any

import yaml

from legwheel_experiments.schemas import require_mapping, require_number, require_positive_number


def validate_wheel_parameter_readback(
    requested_parameters: Sequence[Any], reported_values: Sequence[Any],
) -> None:
    """Compare ROS wire values, retaining parameter names in failure messages.

    Parameter.Type is a Python Enum, whereas ParameterValue.type is an integer.
    Serializing each request gives us integer type codes on both sides. Keeping
    this check independent of ROS imports also makes that boundary testable.
    """
    prefix = "Motor node wheel settings do not match the requested run"
    if len(reported_values) != len(requested_parameters):
        raise RuntimeError(
            f"{prefix}: expected {len(requested_parameters)} parameters, "
            f"received {len(reported_values)}"
        )

    mismatches = []
    for parameter, reported in zip(requested_parameters, reported_values):
        expected = parameter.get_parameter_value()
        if reported.type != expected.type:
            mismatches.append(
                f"{parameter.name}: expected ROS type {expected.type}, received {reported.type}"
            )
            continue
        # Wheel configuration uses only strings and doubles. Read the active
        # message field only after checking its type, including for zero values.
        value_field = "string_value" if isinstance(parameter.value, str) else "double_value"
        expected_value = getattr(expected, value_field)
        reported_value = getattr(reported, value_field)
        if reported_value != expected_value:
            mismatches.append(
                f"{parameter.name}: expected {expected_value!r}, received {reported_value!r}"
            )
    if mismatches:
        raise RuntimeError(f"{prefix}: " + "; ".join(mismatches))


@dataclass(frozen=True)
class WheelControlDefaults:
    """Resolved defaults plus their original contents for reproducible snapshots."""

    motor_config_path: Path
    motor_limits_path: Path
    proportional_gain: float
    integral_gain: float
    maximum_speed_rad_s: float
    motor_config_contents: str | None = None
    motor_limits_contents: str | None = None


def load_wheel_control_defaults(config_directory: Path | None = None) -> WheelControlDefaults:
    """Use the installed package, matching the hardware launch's default paths."""
    if config_directory is None:
        from ament_index_python.packages import get_package_share_directory
        config_directory = Path(get_package_share_directory("legwheel_can")) / "config"
    motor_config_path = config_directory / "motor_config.json"
    motor_limits_path = config_directory / "motor_limits.yaml"
    motor_config_contents = motor_config_path.read_text(encoding="utf-8")
    motor_limits_contents = motor_limits_path.read_text(encoding="utf-8")
    motor = json.loads(motor_config_contents)
    try:
        limits = yaml.safe_load(motor_limits_contents)
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid motor limits YAML: {error}") from error
    if not isinstance(motor, dict) or not isinstance(limits, dict):
        raise ValueError("Motor defaults and limits must contain mappings")
    node_limits = require_mapping(limits, "legwheel_can", "motor limits.legwheel_can")
    parameters = require_mapping(node_limits, "ros__parameters", "motor limits.ros__parameters")
    maximum_speed = require_positive_number(
        parameters.get("wheel_velocity_governor_max_rad_s"),
        "maximum selectable governor speed",
    )
    if maximum_speed > 4.5:
        raise ValueError("The governor selectable upper bound cannot exceed 4.5 rad/s")
    return WheelControlDefaults(
        motor_config_path=motor_config_path,
        motor_limits_path=motor_limits_path,
        proportional_gain=require_positive_number(
            motor.get("wheel_velocity_governor_default_kp"), "default governor P"
        ),
        integral_gain=require_number(
            motor.get("wheel_velocity_governor_default_ki"), "default governor I", minimum=0.0
        ),
        maximum_speed_rad_s=maximum_speed,
        motor_config_contents=motor_config_contents,
        motor_limits_contents=motor_limits_contents,
    )


def ask_float(
    prompt: str, minimum: float | None = None, maximum: float | None = None,
    *, default: float | None = None,
) -> float:
    """Ask for a finite number, optionally accepting a displayed default on Enter."""
    while True:
        raw_value = input(prompt)

        try:
            value = float(default if not raw_value.strip() and default is not None else raw_value)
        except ValueError:
            print("Invalid input. Please enter a valid number.")
            continue

        if not math.isfinite(value):
            print("Value must be finite. Please try again.")
            continue

        if minimum is not None and value < minimum:
            print(f"Value must be at least {minimum}. Please try again.")
            continue

        if maximum is not None and value > maximum:
            print(f"Value must be at most {maximum}. Please try again.")
            continue

        return value


def collect_wheel_control_settings(defaults: WheelControlDefaults) -> dict[str, Any]:
    """Resolve every setting now, so later JSON edits cannot alter this run."""
    while True:
        choice = input(
            "Wheel operation: 1 = pure raw torque, 2 = raw torque with speed cap: "
        ).strip()
        if choice in {"1", "2"}:
            break
        print("Enter 1 or 2.")

    settings = {
        "control_mode": "raw_torque",
        "max_wheel_speed_rad_s": None,
        "velocity_governor_kp": defaults.proportional_gain,
        "velocity_governor_ki": defaults.integral_gain,
    }
    if choice == "1":
        return settings
    settings["control_mode"] = "raw_torque_velocity_capped"
    while True:
        cap = ask_float(
            f"Maximum wheel speed (rad/s, > 0 and <= {defaults.maximum_speed_rad_s}): ",
            minimum=0.0, maximum=defaults.maximum_speed_rad_s,
        )
        if cap > 0.0:
            settings["max_wheel_speed_rad_s"] = cap
            break
        print("The speed cap must be greater than zero.")
    print(
        f"Governor defaults: P={defaults.proportional_gain}, I={defaults.integral_gain}. "
        "These are provisional tuning values. Active braking is disabled."
    )
    while True:
        change = input("Change governor P and I for this run? (y/n): ").strip().lower()
        if change in {"y", "n"}:
            break
        print("Enter y or n.")
    if change == "y":
        while True:
            gain = ask_float(
                f"Governor P (Nm s/rad, > 0) [{defaults.proportional_gain}]: ",
                minimum=0.0, default=defaults.proportional_gain,
            )
            if gain > 0.0:
                settings["velocity_governor_kp"] = gain
                break
            print("P must be greater than zero.")
        settings["velocity_governor_ki"] = ask_float(
            f"Governor I (Nm/rad, >= 0; zero selects P only) [{defaults.integral_gain}]: ",
            minimum=0.0, default=defaults.integral_gain,
        )
    return settings
