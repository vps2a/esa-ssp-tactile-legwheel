import pytest
from dataclasses import asdict
import yaml

from legwheel_experiments.data_recorder import RunRecorder
from legwheel_experiments.schemas import ExperimentConfig, RunConfig, validate_experiment_config


def experiment_config(theta_y_kinematic_sign=-1) -> ExperimentConfig:
    """Return the smallest valid experiment containing the direction mapping."""
    encoder_setup = {
        "port": "/dev/ttyUSB0",
        "baudrate": 9600,
        "ticks_per_legwheel_revolution": 62.5,
        "publish_rate_hz": 100.0,
        "invert_direction": False,
        "zero_on_start": True,
    }
    if theta_y_kinematic_sign is not None:
        encoder_setup["theta_y_kinematic_sign"] = theta_y_kinematic_sign
    return ExperimentConfig(
        schema_version="1.0",
        experiment_id="synthetic",
        datetime_of_creation="2026-10-01T00:00:00+00:00",
        rig_config={
            "beam_radius_m": 0.83,
            "beam_total_length_m": 1.0,
            "pivot_height_m": 0.353,
        },
        leg_config={
            "hip_link_length_m": 0.2,
            "calf_link_length_m": 0.2,
        },
        electronics_hardware={
            "encoder_setup": encoder_setup,
            "camera": {
                "camera_config": {
                    "sensor_front_offset_m": 0.043,
                    "camera_beam_offset_m": 0.054,
                    "camera_height_m": 0.029,
                    "camera_front_angle_deg": 40.0,
                }
            },
        },
        wheel_config={
            "wheel_radius_mm": 50.0,
            "wheel_width_mm": 100.0,
        },
        environment={},
    )


def test_current_geometry_accepts_negative_kinematic_sign():
    validate_experiment_config(experiment_config(-1))


@pytest.mark.parametrize("field", ["sensor_front_offset_m", "camera_beam_offset_m"])
def test_camera_offsets_are_required(field):
    config = experiment_config()
    del config.electronics_hardware["camera"]["camera_config"][field]
    with pytest.raises(ValueError, match=field):
        validate_experiment_config(config)


@pytest.mark.parametrize("invalid_sign", [None, 0, 0.5, 2])
def test_kinematic_sign_is_required_and_discrete(invalid_sign):
    with pytest.raises(ValueError, match="theta_y_kinematic_sign"):
        validate_experiment_config(experiment_config(invalid_sign))


@pytest.mark.parametrize("invalid_width_mm", [0.0, 830.0, 900.0, 1660.0])
def test_wheel_width_must_leave_a_positive_inner_track_radius(invalid_width_mm):
    config = experiment_config()
    config.wheel_config["wheel_width_mm"] = invalid_width_mm
    with pytest.raises(ValueError, match="wheel_width_mm"):
        validate_experiment_config(config)


def test_wheel_width_can_be_just_smaller_than_beam_radius():
    config = experiment_config()
    config.wheel_config["wheel_width_mm"] = 829.0
    validate_experiment_config(config)


def test_recording_saves_validated_values_and_needs_no_separate_encoder_file(tmp_path):
    config = experiment_config()
    source = tmp_path / "experiment.yaml"
    # Simulate editing the source after the CLI has loaded and validated it.
    changed_source = asdict(config)
    changed_source["electronics_hardware"]["encoder_setup"]["ticks_per_legwheel_revolution"] = 83.3
    source.write_text(yaml.safe_dump(changed_source))
    run = RunConfig(
        run_length_loops=1.0,
        leg_parameters={
            "knee_stiffness_nm_per_rad": 10.0,
            "knee_spring_zeroposition_rad": 0.5,
            "knee_damping_nms_per_rad": 1.0,
        },
        wheel_parameters={"commanded_wheel_torque_nm": 2.0, "wheel_torque_ramp_time_sec": 1.0},
    )
    directory = RunRecorder(tmp_path).start_run(run, config, source, 1)
    snapshot = yaml.safe_load((directory / "experiment_config_run_1_snapshot.yaml").read_text())
    assert snapshot == asdict(config)
    assert not list(directory.glob("rotation_encoder_config*"))
