import pytest

from legwheel_experiments.schemas import ExperimentConfig, validate_experiment_config


def experiment_config(theta_y_kinematic_sign=-1) -> ExperimentConfig:
    """Return the smallest valid experiment containing the direction mapping."""
    encoder_setup = {"ticks_per_legwheel_revolution": 62.5}
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
                    "camera_beam_offset_m": -0.043,
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


@pytest.mark.parametrize("invalid_sign", [None, 0, 0.5, 2])
def test_kinematic_sign_is_required_and_discrete(invalid_sign):
    with pytest.raises(ValueError, match="theta_y_kinematic_sign"):
        validate_experiment_config(experiment_config(invalid_sign))
