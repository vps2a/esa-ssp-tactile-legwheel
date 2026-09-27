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

# These are the vendor-facing inputs to legwheel_rgbd. They are intentionally
# opt-in because recording them alongside the stable forwarded topics nearly
# doubles camera bandwidth. They are invaluable when diagnosing driver, QoS, or
# forwarding-rate problems.
RAW_CAMERA_TOPICS = (
    "/legwheel_rgbd/color/image_raw",
    "/legwheel_rgbd/color/camera_info",
    "/legwheel_rgbd/depth/image_raw",
    "/legwheel_rgbd/depth/camera_info",
    "/legwheel_rgbd/gyro_accel/sample",
)


def topics_for_recording(include_raw_camera_topics: bool = False) -> tuple[str, ...]:
    """Return the normal dataset topics plus optional raw camera diagnostics."""
    if include_raw_camera_topics:
        return ROSBAG_TOPICS + RAW_CAMERA_TOPICS
    return ROSBAG_TOPICS


class RosbagRecorder:
    def __init__(
        self,
        run_directory: Path,
        include_raw_camera_topics: bool = False,
    ) -> None:
        self._run_directory = run_directory
        self._bag_directory = run_directory / "rosbag"
        self._topics = topics_for_recording(include_raw_camera_topics)
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
            *self._topics,
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
