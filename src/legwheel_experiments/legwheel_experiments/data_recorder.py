import json
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from legwheel_experiments.state_machine import ExperimentState

import yaml

class RunRecorder:
    def __init__(self, experiment_directory: Path) -> None:
        self.experiment_directory = experiment_directory
        self.run_directory: Path | None = None

    def start_run(self, run_config: dict[str, Any], experiment_config_path: Path) -> Path:    
        self.run_directory = self._create_run(run_config)
        self._change_marker(ExperimentState.RUNNING.name)
        return self.run_directory

    def has_active_run(self) -> bool:
        if self.run_directory is None:
            return False
        
    #TODO Fix the markers so they represent the acutal state machine state

        running_marker = self.run_directory / "RUNNING"
        return running_marker.exists()
    
    def mark_complete(self) -> None:
        self._change_marker(ExperimentState.COMPLETE.name)
    
    def mark_aborted(self, reason: str) -> None:
        self._change_marker(ExperimentState.ABORTING.name)


    def _change_marker(self, new_state: str) -> None:
        if self.run_directory is None:
            raise RuntimeError("No active run to change state.")
    
        #Delete the current marker from the directory if it exists (state markers taken from the ExperimentState enum)
        for state in ExperimentState:
            marker = self.run_directory / state.name
            if marker.exists():
                marker.unlink()
        
        #Creating the new marker
        marker = self.run_directory / new_state
        marker.touch(exist_ok=False)


    def _find_next_run_number(self) -> int:
        run_numbers = []

        for path in self.experiment_directory.glob("run_*"):
            if not path.is_dir():
                continue

            suffix = path.name.removeprefix("run_")

            if suffix.isdigit():
                run_numbers.append(int(suffix))

        return max(run_numbers, default=0) + 1

    def _create_run(self, run_config: dict[str, Any]) -> Path:
        run_number = self._find_next_run_number()
        self.run_directory = (
            self.experiment_directory / f"run_{run_number}"
        )

        self.run_directory.mkdir(parents=False, exist_ok=False)

        run_config_path = self.run_directory / "run_config.yaml"

        with run_config_path.open("w", encoding="utf-8") as file:
            yaml.safe_dump(run_config, file, sort_keys=False)

        #Taking the configuration snapshot and copying it into the run directory
        source = self.experiment_directory / "exp_1_config.yaml"
        destination = (
            self.run_directory / "experiment_config_snapshot.yaml"
        )

        shutil.copy2(source, destination)

        #TODO: Full metadata needs to be created
        #Writing experiment metadata
        metadata = {
            "run_id": self.run_directory.name,
            "experiment_id": self.experiment_directory.name,
            "start_time_utc": datetime.now(timezone.utc).isoformat(),
            "abort_reason": None,
        }

        with (self.run_directory / "metadata.json").open(
            "w",
            encoding="utf-8",
        ) as file:
            json.dump(metadata, file, indent=2)

        return self.run_directory