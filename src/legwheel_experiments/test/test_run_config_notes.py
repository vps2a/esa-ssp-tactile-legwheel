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
        "run_valid": None,
        "post_run_notes": "",
    }


def test_run_notes_must_be_text():
    raw_config = raw_run_config()
    raw_config["notes"] = ["not", "text"]

    with pytest.raises(ValueError, match="notes must be text"):
        make_run_config(raw_config)


@pytest.mark.parametrize("valid", [True, False])
def test_post_run_review_round_trips_as_boolean_and_separate_notes(valid):
    raw_config = {
        **raw_run_config(), "notes": "Before the run",
        "run_valid": valid, "post_run_notes": "After the run",
    }
    saved = yaml.safe_load(yaml.safe_dump(asdict(make_run_config(raw_config))))
    assert saved == raw_config
    assert type(saved["run_valid"]) is bool


@pytest.mark.parametrize("invalid", [0, 1, "false", [], {}])
def test_run_valid_rejects_non_booleans(invalid):
    with pytest.raises(ValueError, match="run_valid"):
        make_run_config({**raw_run_config(), "run_valid": invalid})


def test_post_run_notes_must_be_text():
    with pytest.raises(ValueError, match="post_run_notes"):
        make_run_config({**raw_run_config(), "post_run_notes": ["not text"]})
