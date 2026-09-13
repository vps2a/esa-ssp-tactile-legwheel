import os
import signal
import subprocess
import time
from pathlib import Path

ROSBAG_TOPICS = (
    "/legwheel/joint_states",
    "/wheel/wheel_state",
    "/rotation_encoder/joint_state",
    "/rotation_encoder/ticks",
    "/camera/rgbd/rgb/image_raw",
    "/camera/rgbd/rgb/camera_info",
    "/camera/rgbd/depth/image_raw",
    "/camera/rgbd/depth/camera_info",
    "/camera/imu",
    "/legwheel/motor_status",
    "/legwheel/spring_zero_position",
    "/legwheel/spring_constant",
    "/legwheel/damping_constant",
    "/wheel/requested_torque",
)


class RosbagRecorder:
    def __init__(self, run_directory: Path) -> None:
        self._run_directory = run_directory
        self._bag_directory = run_directory / "rosbag"
        self._process: subprocess.Popen | None = None
        self._log_file = None

    def start(self) -> Path:
        if self._bag_directory.exists():
            raise RuntimeError(
                f"Rosbag output already exists: {self._bag_directory}"
            )

        log_path = self._run_directory / "rosbag_recorder.log"
        self._log_file = log_path.open("w", encoding="utf-8")

        command = [
            "ros2", "bag", "record",
            "--storage", "mcap",
            "--output", str(self._bag_directory),
            "--topics",
            *ROSBAG_TOPICS,
        ]
        
        self._process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,  # rosbag must not share CLI keyboard input
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        time.sleep(0.5)
        if self._process.poll() is not None:
            raise RuntimeError(
                "Rosbag recorder exited during startup; "
                f"inspect {log_path}"
            )

        return self._bag_directory

    def stop(self) -> None:
        if self._process is None:
            return

        if self._process.poll() is None:
            # SIGINT causes rosbag to flush indexes and write metadata.yaml.
            os.killpg(os.getpgid(self._process.pid), signal.SIGINT)
            try:
                self._process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
                self._process.wait(timeout=5.0)

        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None