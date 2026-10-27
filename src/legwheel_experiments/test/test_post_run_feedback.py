"""Post-run feedback must follow successful recording shutdown and preserve setup."""

from dataclasses import asdict, replace
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

from legwheel_experiments import cli
from legwheel_experiments.data_recorder import RunRecorder
from legwheel_experiments.schemas import make_run_config
from legwheel_experiments.state_machine import ExperimentState
from legwheel_experiments.wheel_control import WheelControlDefaults


@pytest.fixture
def run_config():
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
        },
        "notes": "Original pre-run notes",
    })


@pytest.mark.parametrize("answer,valid", [(" Y ", True), ("n", False)])
@pytest.mark.parametrize("with_notes", [False, True])
def test_feedback_is_saved_without_changing_run_parameters(
    monkeypatch, tmp_path, run_config, answer, valid, with_notes,
):
    recorder = RunRecorder(tmp_path)
    recorder.run_directory = tmp_path
    responses = ["invalid", answer, "invalid", "y" if with_notes else "n"]
    if with_notes:
        responses.append("  Wheel slipped near the rock.  ")
    responses = iter(responses)
    monkeypatch.setattr("builtins.input", lambda _: next(responses))

    cli.record_post_run_feedback(recorder, run_config)

    saved = yaml.safe_load((tmp_path / "run_config.yaml").read_text())
    assert saved == {
        **asdict(run_config), "run_valid": valid,
        "post_run_notes": "Wheel slipped near the rock." if with_notes else "",
    }
    assert type(saved["run_valid"]) is bool
    assert run_config.run_valid is None  # The frozen input was not mutated.


@pytest.mark.parametrize("interruption", [EOFError, KeyboardInterrupt])
def test_validity_survives_interruption_during_notes(
    monkeypatch, tmp_path, run_config, interruption,
):
    recorder = RunRecorder(tmp_path)
    recorder.run_directory = tmp_path

    def respond(prompt):
        if prompt.startswith("Was the run valid?"):
            return "n"
        raise interruption()

    monkeypatch.setattr("builtins.input", respond)
    with pytest.raises(interruption):
        cli.record_post_run_feedback(recorder, run_config)
    saved = yaml.safe_load((tmp_path / "run_config.yaml").read_text())
    assert saved["run_valid"] is False
    assert saved["post_run_notes"] == ""
    assert saved["notes"] == run_config.notes


@pytest.mark.parametrize("outcome", ["complete", "aborted", "interrupted", "stop_failed"])
def test_cli_prompts_only_after_normal_completion_and_successful_rosbag_stop(
    monkeypatch, tmp_path, run_config, outcome,
):
    # Fake hardware/runtime boundaries, but exercise the actual CLI lifecycle.
    # No ROS installation, executor activity, or real motor motion is needed.
    events = []
    ros = ModuleType("rclpy")
    ros.__path__ = []
    ros.init = Mock()
    ros.ok = lambda: True
    ros.shutdown = lambda: events.append("ros_shutdown")
    executors = ModuleType("rclpy.executors")
    executors.SingleThreadedExecutor = Mock(return_value=Mock())
    runner_module = ModuleType("legwheel_experiments.experiment_runner_node")

    class AbortedError(RuntimeError):
        pass

    node = Mock()
    node.wait_for_preflight.return_value = True
    node.experiment_state.side_effect = [ExperimentState.WAITING_FOR_ARM, ExperimentState.ARMED]
    node.submit_arm_request.return_value = True
    node.wait_for_arm_result.return_value = (True, "Armed")
    node.abort_reason.return_value = "simulated abort"
    runner_module.ExperimentRunnerNode = Mock(return_value=node)
    runner_module.ExperimentAbortedError = AbortedError
    monkeypatch.setitem(sys.modules, "rclpy", ros)
    monkeypatch.setitem(sys.modules, "rclpy.executors", executors)
    monkeypatch.setitem(sys.modules, "legwheel_experiments.experiment_runner_node", runner_module)

    recorder = Mock()
    recorder.start_run.return_value = tmp_path
    recorder.has_active_run.return_value = True
    recorder.mark_complete.side_effect = lambda: events.append("complete_marker")
    bag = Mock()
    bag.start.return_value = tmp_path / "rosbag"

    def stop_bag():
        events.append("bag_stop")
        if outcome == "stop_failed":
            raise RuntimeError("simulated recording shutdown failure")

    bag.stop.side_effect = stop_bag
    adjusted_run = replace(run_config, leg_parameters={
        **run_config.leg_parameters, "knee_spring_zeroposition_rad": 0.7,
    })

    def run_after_arm(current_run, experiment, *, spring_zero_position_callback):
        spring_zero_position_callback(current_run)
        if outcome == "aborted":
            raise AbortedError("simulated abort")
        if outcome == "interrupted":
            raise KeyboardInterrupt()
        events.append("leg_straightened")

    node.run_experiment_after_arm.side_effect = run_after_arm
    monkeypatch.setattr(cli, "parse_arguments", lambda: SimpleNamespace(
        experiment_config=tmp_path / "experiment.yaml",
    ))
    monkeypatch.setattr(cli, "load_experiment_config", lambda _: SimpleNamespace(experiment_id="test"))
    monkeypatch.setattr(cli, "load_wheel_control_defaults", lambda: WheelControlDefaults(
        Path("motor.json"), Path("limits.yaml"), 0.5, 0.1, 4.5,
    ))
    monkeypatch.setattr(cli, "collect_run_configuration", lambda _: asdict(run_config))
    monkeypatch.setattr(cli, "print_run_configuration", lambda *_: None)
    monkeypatch.setattr(cli, "confirm_run_creation", lambda: True)
    monkeypatch.setattr(cli, "choose_run_number", lambda _: (1, False))
    monkeypatch.setattr(cli, "RunRecorder", Mock(return_value=recorder))
    monkeypatch.setattr(cli, "RosbagRecorder", Mock(return_value=bag))
    monkeypatch.setattr(cli, "adjust_spring_zero_position", lambda *_: adjusted_run)
    monkeypatch.setattr(cli, "confirm_experiment_start", lambda: None)

    answers = iter(["", "", "ARM", "", "n", "y", "Post-run observation"])

    def respond(prompt):
        if prompt.startswith("Was the run valid?"):
            events.append("validity_prompt")
        return next(answers)

    monkeypatch.setattr("builtins.input", respond)
    cli.main()

    bag.stop.assert_called_once()
    if outcome == "complete":
        assert events.index("leg_straightened") < events.index("bag_stop")
        assert events.index("bag_stop") < events.index("ros_shutdown") < events.index("validity_prompt")
        recorder.mark_complete.assert_called_once()
        recorder.mark_aborted.assert_not_called()
        saved_run = recorder.update_run_config.call_args.args[0]
        assert saved_run.run_valid is False
        assert saved_run.post_run_notes == "Post-run observation"
        assert saved_run.notes == run_config.notes
        assert saved_run.leg_parameters == adjusted_run.leg_parameters
    else:
        assert "validity_prompt" not in events
        recorder.update_run_config.assert_not_called()
        if outcome in {"aborted", "interrupted"}:
            recorder.mark_aborted.assert_called_once()
            recorder.mark_complete.assert_not_called()
