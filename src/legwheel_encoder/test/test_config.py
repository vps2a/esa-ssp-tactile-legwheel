"""Protect the single configuration source and the meaning of published ticks."""

import math

import numpy as np
import pytest
import yaml

from legwheel_encoder.config import DriverTickCounter, EncoderCalibration, EncoderConfig


def settings():
    return {
        "port": "/dev/ttyUSB0",
        "baudrate": 9600,
        "ticks_per_legwheel_revolution": 62.5,
        "publish_rate_hz": 100.0,
        "invert_direction": False,
        "zero_on_start": True,
        "theta_y_kinematic_sign": -1,
    }


@pytest.mark.parametrize("field", list(settings()))
def test_every_driver_setting_is_required(field):
    values = settings()
    del values[field]
    with pytest.raises(ValueError, match=field):
        EncoderConfig.from_setup(values)


@pytest.mark.parametrize("invalid", [None, True, "62.5", 0, -1, math.nan, math.inf])
def test_calibration_rejects_missing_nonfinite_and_nonnumeric_values(invalid):
    with pytest.raises(ValueError, match="ticks_per_legwheel_revolution"):
        EncoderCalibration.from_experiment_dict(
            {
                "electronics_hardware": {
                    "encoder_setup": {"ticks_per_legwheel_revolution": invalid}
                }
            }
        )


@pytest.mark.parametrize(
    "field,invalid",
    [
        ("baudrate", True),
        ("baudrate", 9600.5),
        ("baudrate", 0),
        ("port", ""),
        ("publish_rate_hz", 0),
        ("publish_rate_hz", math.nan),
        ("invert_direction", "false"),
        ("zero_on_start", 1),
        ("theta_y_kinematic_sign", True),
        ("theta_y_kinematic_sign", 0),
    ],
)
def test_driver_settings_validate_types_and_ranges(field, invalid):
    values = settings()
    values[field] = invalid
    with pytest.raises(ValueError, match=field):
        EncoderConfig.from_setup(values)


def test_driver_loads_the_experiment_instead_of_using_a_fallback(tmp_path):
    values = settings()
    values.update(port="/dev/encoder-test", ticks_per_legwheel_revolution=125.0)
    path = tmp_path / "experiment.yaml"
    path.write_text(yaml.safe_dump({"electronics_hardware": {"encoder_setup": values}}))
    config = EncoderConfig.from_yaml(path)
    assert config.port == "/dev/encoder-test"
    assert config.calibration.motion_angle(125) == pytest.approx(2 * math.pi)


@pytest.mark.parametrize(
    "zero_on_start,expected", [(True, [0, 5]), (False, [120, 125])]
)
@pytest.mark.parametrize("invert_direction", [False, True])
def test_startup_zeroing_and_direction_are_applied_once(
    zero_on_start, expected, invert_direction
):
    counter = DriverTickCounter(
        zero_on_start=zero_on_start, invert_direction=invert_direction
    )
    direction = -1 if invert_direction else 1
    counts = [counter.update(ticks) for ticks in [120, 125]]
    assert counts == [direction * value for value in expected]
    # Recording-relative reconstruction is independent of driver startup zero.
    calibration = EncoderCalibration(62.5)
    np.testing.assert_allclose(
        calibration.motion_angle(np.asarray(counts), counts[0]),
        [0, direction * 5 * 2 * math.pi / 62.5],
    )
