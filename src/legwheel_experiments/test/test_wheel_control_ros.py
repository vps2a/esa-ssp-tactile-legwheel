"""Control handshakes and runtime guards, using ROS message/parameter types."""

from types import SimpleNamespace
import time

import pytest

pytest.importorskip("rclpy")
from rclpy.parameter import Parameter
from rclpy.task import Future

from legwheel_experiments.experiment_runner_node import ExperimentRunnerNode
from legwheel_experiments.state_machine import ExperimentState
from legwheel_experiments.schemas import RunConfig


def completed(value):
    future = Future()
    future.set_result(value)
    return future


class MotorParameterClient:
    def __init__(self, *, accept=True, wrong_readback=False):
        self.accept = accept
        self.wrong_readback = wrong_readback
        self.parameters = {}

    def wait_for_services(self, timeout_sec):
        return True

    def set_parameters_atomically(self, parameters):
        self.parameters = {parameter.name: parameter for parameter in parameters}
        return completed(SimpleNamespace(result=SimpleNamespace(
            successful=self.accept, reason="motors enabled" if not self.accept else ""
        )))

    def get_parameters(self, names):
        values = [self.parameters[name].get_parameter_value() for name in names]
        if self.wrong_readback:
            values[1] = Parameter("cap", value=4.0).get_parameter_value()
        return completed(SimpleNamespace(values=values))


def make_runner(client):
    runner = object.__new__(ExperimentRunnerNode)
    runner._wheel_parameter_client = client
    runner._reported_wheel_control = None
    runner._wheel_control_received_at_s = None
    runner._state_machine = SimpleNamespace(state=ExperimentState.RUNNING)
    return runner


def run_configuration():
    return RunConfig(1.0, {
        "knee_stiffness_nm_per_rad": 10.0, "knee_spring_zeroposition_rad": 0.5,
        "knee_damping_nms_per_rad": 1.0,
    }, {
        "commanded_wheel_torque_nm": 2.0, "wheel_torque_ramp_time_sec": 1.0,
        "control_mode": "raw_torque_velocity_capped", "max_wheel_speed_rad_s": 3.0,
        "velocity_governor_kp": 0.5, "velocity_governor_ki": 0.1,
    })


def test_atomic_configuration_requires_matching_readback_and_live_status():
    client = MotorParameterClient()
    runner = make_runner(client)
    runner.configure_wheel_control(run_configuration())
    assert len(client.parameters) == 5
    assert runner._wheel_configuration_confirmed
    assert not runner._wheel_control_ready()
    runner._reported_wheel_control = {
        **runner._expected_wheel_control, "fault": "", "active_braking_enabled": False,
        "velocity_valid": True,
    }
    runner._wheel_control_received_at_s = time.monotonic()
    assert runner._wheel_control_ready()


@pytest.mark.parametrize("client", [
    MotorParameterClient(accept=False), MotorParameterClient(wrong_readback=True),
])
def test_rejected_or_mismatched_settings_never_confirm(client):
    runner = make_runner(client)
    with pytest.raises(RuntimeError):
        runner.configure_wheel_control(run_configuration())
    assert not runner._wheel_configuration_confirmed


def test_overspeed_and_one_failed_read_do_not_abort_but_fault_or_restart_do():
    runner = make_runner(MotorParameterClient())
    runner.configure_wheel_control(run_configuration())
    runner._reported_wheel_control = {
        **runner._expected_wheel_control, "fault": "", "active_braking_enabled": False,
        "velocity_valid": False, "measured_speed_rad_s": 5.0,
    }
    runner._wheel_control_received_at_s = time.monotonic()
    failures = []
    runner._abort_for_runtime_failure = lambda reason, source: failures.append(reason)
    runner._wheel_control_guard()
    assert failures == []
    runner._reported_wheel_control["fault"] = "velocity feedback lost"
    runner._wheel_control_guard()
    assert failures == ["velocity feedback lost"]
    runner._reported_wheel_control["fault"] = ""
    runner._reported_wheel_control["session_id"] = "a-different-run"
    runner._wheel_control_guard()
    assert len(failures) == 2
