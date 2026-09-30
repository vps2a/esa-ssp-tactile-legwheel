from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil

import yaml

from legwheel_experiments.camera_topics import CAMERA_DRIVER_CONFIG_PATH
from legwheel_experiments.schemas import (
    ExperimentConfig,
    RunConfig,
    validate_experiment_config,
    validate_run_config,
)
from legwheel_experiments.state_machine import ExperimentState


ADDITIONAL_CONFIG_SNAPSHOTS = (
    (
        Path("src/legwheel_encoder/config/rotation_encoder.yaml"),
        "rotation_encoder_config",
    ),
    (Path("src/legwheel_can/config/motor_config.json"), "motor_config"),
    (Path("src/legwheel_can/config/motor_ids.yaml"), "motor_ids"),
    (Path("src/legwheel_can/config/motor_limits.yaml"), "motor_limits"),
)


class RunRecorder:
    """Create a run directory and preserve the validated configuration inputs."""

    def __init__(self, experiment_directory: Path) -> None:
        self.experiment_directory = experiment_directory
        self.run_directory: Path | None = None

    def start_run(
        self,
        run_config: RunConfig,
        experiment_config: ExperimentConfig,
        experiment_config_path: Path,
        run_number: int,
        overwrite_existing: bool = False,
    ) -> Path:
        """Validate and create the selected run directory and configuration snapshot."""
        # This protects callers other than the CLI from recording unchecked data.
        validate_run_config(run_config)
        validate_experiment_config(experiment_config)

        self.run_directory = self._create_run(
            run_config=run_config,
            experiment_config=experiment_config,
            experiment_config_path=experiment_config_path,
            run_number=run_number,
            overwrite_existing=overwrite_existing,
        )
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
        """Mark the run aborted and retain its cause for later diagnosis."""
        if self.run_directory is None:
            raise RuntimeError("No active run to mark as aborted.")

        metadata_path = self.run_directory / "metadata.json"
        temporary_path = self.run_directory / "metadata.json.tmp"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        metadata["abort_reason"] = reason
        metadata["end_time_utc"] = datetime.now(timezone.utc).isoformat()

        try:
            with temporary_path.open("w", encoding="utf-8") as file:
                json.dump(metadata, file, indent=2)
            temporary_path.replace(metadata_path)
        finally:
            if temporary_path.exists():
                temporary_path.unlink()

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


    def find_next_run_number(self) -> int:
        """Return the first run number after the highest existing run directory."""
        run_numbers = []

        for path in self.experiment_directory.glob("run_*"):
            if not path.is_dir():
                continue

            suffix = path.name.removeprefix("run_")

            if suffix.isdigit():
                run_numbers.append(int(suffix))

        return max(run_numbers, default=0) + 1

    def run_directory_for_number(self, run_number: int) -> Path:
        """Return the only directory that may be created or overwritten for a run."""
        if not isinstance(run_number, int) or isinstance(run_number, bool) or run_number < 1:
            raise ValueError("Run number must be a positive integer.")
        return self.experiment_directory / f"run_{run_number}"

    def update_run_config(self, run_config: RunConfig) -> None:
        """Atomically replace the saved run configuration after an operator adjustment."""
        validate_run_config(run_config)
        if self.run_directory is None:
            raise RuntimeError("No active run directory to update.")

        run_config_path = self.run_directory / "run_config.yaml"
        temporary_path = self.run_directory / "run_config.yaml.tmp"

        try:
            with temporary_path.open("w", encoding="utf-8") as file:
                yaml.safe_dump(asdict(run_config), file, sort_keys=False)
            # replace() is atomic on one filesystem, so a crash cannot leave a partial YAML file.
            temporary_path.replace(run_config_path)
        finally:
            # Remove an unfinished temporary file if writing or replacing failed.
            if temporary_path.exists():
                temporary_path.unlink()

    def _create_run(
        self,
        run_config: RunConfig,
        experiment_config: ExperimentConfig,
        experiment_config_path: Path,
        run_number: int,
        overwrite_existing: bool,
    ) -> Path:
        """Write the run configuration and copy its source configuration files."""

        target_directory = self.run_directory_for_number(run_number)

        if not experiment_config_path.is_file():
            raise FileNotFoundError(
                f"Experiment config snapshot source does not exist: {experiment_config_path}"
            )

        # Validate every required source before deleting a confirmed older run.
        camera_topics_path = Path(
            "src/legwheel_experiments/legwheel_experiments/camera_topics.py"
        )
        camera_driver_config_path = (
            Path(CAMERA_DRIVER_CONFIG_PATH)
            if CAMERA_DRIVER_CONFIG_PATH is not None
            else None
        )
        if not camera_topics_path.is_file():
            raise FileNotFoundError(
                f"Camera topic mapping does not exist: {camera_topics_path}"
            )
        if (
            camera_driver_config_path is not None
            and not camera_driver_config_path.is_file()
        ):
            raise FileNotFoundError(
                "Camera driver config file does not exist: "
                f"{camera_driver_config_path}"
            )
        for source_path, _ in ADDITIONAL_CONFIG_SNAPSHOTS:
            if not source_path.is_file():
                raise FileNotFoundError(
                    f"Configuration snapshot source does not exist: {source_path}"
                )

        if target_directory.exists():
            if not overwrite_existing:
                raise FileExistsError(
                    f"Run directory already exists: {target_directory}"
                )
            if target_directory.is_symlink() or not target_directory.is_dir():
                raise RuntimeError(
                    f"Refusing to overwrite unexpected path: {target_directory}"
                )
            # This target is derived only from a validated positive run number.
            shutil.rmtree(target_directory)

        self.run_directory = target_directory
        self.run_directory.mkdir(parents=False, exist_ok=False)

        run_config_path = self.run_directory / "run_config.yaml"
        with run_config_path.open("w", encoding="utf-8") as file:
            # asdict records the immutable dataclass as ordinary YAML data.
            yaml.safe_dump(asdict(run_config), file, sort_keys=False)

        # Take the configuration snapshots and copy them into the run directory.
        destination = (
            self.run_directory / f"experiment_config_run_{run_number}_snapshot.yaml"
        )
        shutil.copy2(experiment_config_path, destination)

        camera_topics_destination = (
            self.run_directory / f"camera_topics_run_{run_number}_snapshot.py"
        )
        # Copy the topic interface and optional driver configuration used by
        # this run so the dataset remains reproducible after later code changes.
        shutil.copy2(camera_topics_path, camera_topics_destination)
        if camera_driver_config_path is not None:
            camera_driver_config_destination = self.run_directory / (
                f"camera_driver_config_run_{run_number}_snapshot"
                f"{camera_driver_config_path.suffix}"
            )
            shutil.copy2(
                camera_driver_config_path,
                camera_driver_config_destination,
            )
        for source_path, snapshot_name in ADDITIONAL_CONFIG_SNAPSHOTS:
            destination = self.run_directory / (
                f"{snapshot_name}_run_{run_number}_snapshot{source_path.suffix}"
            )
            shutil.copy2(source_path, destination)

        #TODO: Full metadata needs to be created
        #Writing experiment metadata
        metadata = {
            "run_id": self.run_directory.name,
            "experiment_id": experiment_config.experiment_id,
            "start_time_utc": datetime.now(timezone.utc).isoformat(),
            "abort_reason": None,
            "experiment_config_source": str(experiment_config_path),
            "experiment_config_filename": experiment_config_path.name,
        }

        with (self.run_directory / "metadata.json").open(
            "w",
            encoding="utf-8",
        ) as file:
            json.dump(metadata, file, indent=2)

        return self.run_directory
