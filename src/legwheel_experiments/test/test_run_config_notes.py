from dataclasses import asdict

import pytest
import yaml

from legwheel_experiments.schemas import make_run_config


def raw_run_config() -> dict:
    return {
        "run_length_loops": 1.0,
        "leg_parameters": {
            "knee_stiffness_nm_per_rad": 10.0,
            "knee_spring_zeroposition_rad": 0.5,
            "knee_damping_nms_per_rad": 1.0,
        },
        "wheel_parameters": {
            "commanded_wheel_torque_nm": 2.0,
            "wheel_torque_ramp_time_sec": 1.0,
        },
    }


def test_run_notes_are_serialized_with_the_run_configuration():
    raw_config = raw_run_config()
    raw_config["notes"] = "Test surface was wet."

    run_config = make_run_config(raw_config)

    assert yaml.safe_load(yaml.safe_dump(asdict(run_config))) == {
        **raw_config,
    }


def test_run_notes_must_be_text():
    raw_config = raw_run_config()
    raw_config["notes"] = ["not", "text"]

    with pytest.raises(ValueError, match="notes must be text"):
        make_run_config(raw_config)
