"""Exercise real input validation and serialization without a ROS installation."""

from dataclasses import asdict
import json
from pathlib import Path

import pytest
import yaml

from legwheel_experiments.schemas import make_run_config
from legwheel_experiments.wheel_control import (
    WheelControlDefaults, collect_wheel_control_settings, load_wheel_control_defaults,
)


@pytest.fixture
def defaults():
    return WheelControlDefaults(Path("motor.json"), Path("limits.yaml"), 0.5, 0.1, 4.5)


def answer(monkeypatch, responses):
    responses = iter(responses)
    monkeypatch.setattr("builtins.input", lambda _: next(responses))


def run_with_settings(settings):
    return make_run_config({
        "run_length_loops": 1.0,
        "leg_parameters": {
            "knee_stiffness_nm_per_rad": 10.0,
            "knee_spring_zeroposition_rad": 0.5,
            "knee_damping_nms_per_rad": 1.0,
        },
        "wheel_parameters": {
            "commanded_wheel_torque_nm": 2.0,
            "wheel_torque_ramp_time_sec": 1.0,
            **settings,
        },
    })


def test_pure_mode_records_inactive_defaults_without_tuning_questions(monkeypatch, defaults):
    answer(monkeypatch, ["invalid", "1"])
    settings = collect_wheel_control_settings(defaults)
    assert settings == {
        "control_mode": "raw_torque", "max_wheel_speed_rad_s": None,
        "velocity_governor_kp": 0.5, "velocity_governor_ki": 0.1,
    }
    run_with_settings(settings)


def test_capped_mode_saves_resolved_defaults(monkeypatch, defaults):
    answer(monkeypatch, ["2", "nan", "inf", "0", "4.6", "4.5", "n"])
    run = run_with_settings(collect_wheel_control_settings(defaults))
    saved = yaml.safe_load(yaml.safe_dump(asdict(run)))
    assert saved["wheel_parameters"]["max_wheel_speed_rad_s"] == 4.5
    assert saved["wheel_parameters"]["velocity_governor_kp"] == 0.5
    assert saved["wheel_parameters"]["velocity_governor_ki"] == 0.1


def test_override_can_select_p_only_without_changing_defaults(monkeypatch, defaults):
    answer(monkeypatch, ["2", "3", "y", "0", "nan", "0.75", "-1", "0"])
    settings = collect_wheel_control_settings(defaults)
    run_with_settings(settings)
    assert settings["velocity_governor_kp"] == 0.75
    assert settings["velocity_governor_ki"] == 0.0
    assert defaults.proportional_gain == 0.5


def test_enter_retains_each_displayed_gain_default(monkeypatch, defaults):
    answer(monkeypatch, ["2", "3", "y", "", ""])
    settings = collect_wheel_control_settings(defaults)
    assert settings["velocity_governor_kp"] == 0.5
    assert settings["velocity_governor_ki"] == 0.1


@pytest.mark.parametrize("field,value", [
    ("control_mode", "velocity_pid"), ("control_mode", []),
    ("max_wheel_speed_rad_s", 0), ("max_wheel_speed_rad_s", 4.50001),
    ("max_wheel_speed_rad_s", float("nan")), ("max_wheel_speed_rad_s", True),
    ("velocity_governor_kp", 0), ("velocity_governor_ki", -1),
    ("velocity_governor_kp", float("inf")), ("velocity_governor_ki", False),
])
def test_invalid_run_settings_rejected(field, value):
    settings = {
        "control_mode": "raw_torque_velocity_capped", "max_wheel_speed_rad_s": 3.0,
        "velocity_governor_kp": 0.5, "velocity_governor_ki": 0.1,
    }
    settings[field] = value
    with pytest.raises(ValueError):
        run_with_settings(settings)


def test_legacy_run_stays_pure_and_does_not_acquire_current_gains():
    assert "control_mode" not in run_with_settings({}).wheel_parameters


def test_loads_exact_configuration_paths(tmp_path):
    (tmp_path / "motor_config.json").write_text(json.dumps({
        "wheel_velocity_governor_default_kp": 0.75,
        "wheel_velocity_governor_default_ki": 0.2,
    }))
    (tmp_path / "motor_limits.yaml").write_text(yaml.safe_dump({
        "legwheel_can": {"ros__parameters": {"wheel_velocity_governor_max_rad_s": 4.5}},
    }))
    loaded = load_wheel_control_defaults(tmp_path)
    assert loaded.motor_config_path == tmp_path / "motor_config.json"
    assert loaded.motor_limits_path == tmp_path / "motor_limits.yaml"
    assert loaded.proportional_gain == 0.75
    assert loaded.integral_gain == 0.2
