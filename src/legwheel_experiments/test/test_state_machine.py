import pytest

from legwheel_experiments.state_machine import (
    ExperimentState,
    ExperimentStateMachine,
)


def test_armed_can_transition_to_running():
    state_machine = ExperimentStateMachine()

    state_machine.transition_to(ExperimentState.PREFLIGHT)
    state_machine.transition_to(ExperimentState.WAITING_FOR_ARM)
    state_machine.transition_to(ExperimentState.ARMED)
    state_machine.transition_to(ExperimentState.RUNNING)

    assert state_machine.state is ExperimentState.RUNNING


def test_waiting_for_arm_cannot_transition_directly_to_running():
    state_machine = ExperimentStateMachine()

    state_machine.transition_to(ExperimentState.PREFLIGHT)
    state_machine.transition_to(ExperimentState.WAITING_FOR_ARM)

    with pytest.raises(RuntimeError):
        state_machine.transition_to(ExperimentState.RUNNING)
