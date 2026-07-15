from enum import Enum, auto

class ExperimentState(Enum):
    """Enum representing the different states of the experiment."""
    INITIALIZING = auto()
    PREFLIGHT = auto()
    WAITING_FOR_ARM = auto()
    ARMED = auto()
    RUNNING = auto()
    STOPPING = auto()
    COMPLETE = auto()
    ABORTING = auto()
    SAFE = auto()


ALLOWED_TRANSITIONS = {
    ExperimentState.INITIALIZING: {
        ExperimentState.PREFLIGHT,
        ExperimentState.ABORTING,
    },
    ExperimentState.PREFLIGHT: {
        ExperimentState.WAITING_FOR_ARM,
        ExperimentState.ABORTING,
    },
    ExperimentState.WAITING_FOR_ARM: {
        ExperimentState.ARMED,
        ExperimentState.ABORTING,
    },
    ExperimentState.ARMED: {
        ExperimentState.RUNNING,
        ExperimentState.ABORTING,
    },
    ExperimentState.RUNNING: {
        ExperimentState.STOPPING,
        ExperimentState.ABORTING,
    },
    ExperimentState.STOPPING: {
        ExperimentState.COMPLETE,
        ExperimentState.ABORTING,
    },
    ExperimentState.ABORTING: {
        ExperimentState.SAFE,
    },
}

class ExperimentStateMachine:
    def __init__(self) -> None:
        self._state = ExperimentState.INITIALIZING
        self._abort_reason: str | None = None

    @property
    def state(self) -> ExperimentState:
        return self._state

    @property
    def abort_reason(self) -> str | None:
        return self._abort_reason

    def transition_to(self, new_state: ExperimentState) -> None:
        permitted = ALLOWED_TRANSITIONS.get(self._state, set())

        if new_state not in permitted:
            raise RuntimeError(
                f"Invalid transition: "
                f"{self._state.name} -> {new_state.name}"
            )

        print(f"[SM] State: {self._state.name} -> {new_state.name}")
        self._state = new_state

    def abort(self, reason: str) -> None:
        self._abort_reason = reason

        print(f"[SM] Aborting experiment due to: {reason}")

        if self._state not in {
            ExperimentState.ABORTING,
            ExperimentState.SAFE,
        }:
            self.transition_to(ExperimentState.ABORTING)