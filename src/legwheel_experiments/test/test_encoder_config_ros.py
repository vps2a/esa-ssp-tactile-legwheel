"""A reported encoder configuration must match before recording can proceed."""

import json
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from std_msgs.msg import String

from legwheel_encoder.config import EncoderConfig
from legwheel_experiments.experiment_runner_node import ExperimentRunnerNode


def test_driver_mismatch_and_invalid_reports_revoke_a_matching_configuration():
    settings = {
        "port": "/dev/ttyUSB0",
        "baudrate": 9600,
        "ticks_per_legwheel_revolution": 62.5,
        "publish_rate_hz": 100.0,
        "invert_direction": False,
        "zero_on_start": True,
        "theta_y_kinematic_sign": -1,
    }
    runner = object.__new__(ExperimentRunnerNode)
    runner._expected_encoder_config = EncoderConfig.from_setup(settings)
    runner._reported_encoder_config = None
    warnings = []
    runner.get_logger = lambda: SimpleNamespace(warning=warnings.append)
    assert not runner._encoder_configuration_matches()
    runner._encoder_config_callback(String(data=json.dumps(settings)))
    assert runner._encoder_configuration_matches()
    settings["ticks_per_legwheel_revolution"] = 83.3
    runner._encoder_config_callback(String(data=json.dumps(settings)))
    assert not runner._encoder_configuration_matches()
    runner._encoder_config_callback(String(data="invalid JSON"))
    assert runner._reported_encoder_config is None
    assert not runner._encoder_configuration_matches()
    assert len(warnings) == 1
