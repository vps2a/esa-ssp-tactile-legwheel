#!/usr/bin/env python3

import json
import math
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import Optional, TextIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class WheelControllerNode(Node):
    """Keyboard raw-torque controller for the wheel motor."""

    def __init__(self) -> None:
        super().__init__("wheel_controller_node")

        self.declare_parameter("motor_config_json_path", "")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("torque_step_nm", 0.1)
        self.declare_parameter("torque_limit_nm", 8.0)
        self.declare_parameter("key_release_timeout_s", 0.4)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.torque_step_nm = float(self.get_parameter("torque_step_nm").value)
        self.torque_limit_nm = float(self.get_parameter("torque_limit_nm").value)
        self.key_release_timeout_s = float(
            self.get_parameter("key_release_timeout_s").value
        )
        self.motor_config_json_path = str(
            self.get_parameter("motor_config_json_path").value
        )

        self.ramp_time_s, self.max_torque_nm = self._load_motor_config()
        self._validate_parameters()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._direction = 0
        self._last_drive_key_time: Optional[float] = None
        self._current_torque_nm = 0.0
        self._last_update_time = time.monotonic()

        self.requested_torque_pub = self.create_publisher(
            Float64, "/wheel/requested_torque", 10
        )
        self.update_timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._update_requested_torque
        )

        self.get_logger().info(
            "Wheel controller ready: hold 'w' for forward, 's' for reverse, "
            "'+'/'-' to adjust max torque."
        )
        self.get_logger().info(
            "Initial max_torque=%.3f Nm, ramp_time=%.3f s."
            % (self.max_torque_nm, self.ramp_time_s)
        )

        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._publish_torque(0.0)
        if self._input_thread.is_alive():
            self._input_thread.join(timeout=1.0)

    def _load_motor_config(self):
        if not self.motor_config_json_path:
            raise ValueError("motor_config_json_path parameter is required.")

        path = Path(self.motor_config_json_path)
        with path.open("r", encoding="utf-8") as config_file:
            config = json.load(config_file)

        return (
            float(config["wheel_torque_rampup_time"]),
            float(config["default_max_torque"]),
        )

    def _validate_parameters(self) -> None:
        values = [
            self.publish_rate_hz,
            self.torque_step_nm,
            self.torque_limit_nm,
            self.key_release_timeout_s,
            self.ramp_time_s,
            self.max_torque_nm,
        ]
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("Wheel controller numeric parameters must be finite and positive.")
        if self.max_torque_nm > self.torque_limit_nm:
            self.get_logger().warning(
                "default_max_torque %.3f exceeds torque_limit %.3f; clamping."
                % (self.max_torque_nm, self.torque_limit_nm)
            )
            self.max_torque_nm = self.torque_limit_nm

    def _input_loop(self) -> None:
        input_stream = self._open_command_input()
        close_stream = input_stream is not sys.stdin

        try:
            fd = input_stream.fileno()
            if not input_stream.isatty():
                self.get_logger().error(
                    "Wheel controller needs an interactive terminal for key monitoring."
                )
                return
            old_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            try:
                while not self._stop_event.is_set():
                    readable, _, _ = select.select([input_stream], [], [], 0.05)
                    if not readable:
                        continue

                    character = input_stream.read(1)
                    if character == "":
                        self.get_logger().warning("Wheel controller input closed.")
                        return

                    self._handle_key(character)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        finally:
            if close_stream:
                input_stream.close()

    def _open_command_input(self) -> TextIO:
        try:
            tty_stream = open("/dev/tty", "r", encoding="utf-8")
            self.get_logger().info("Reading wheel commands from /dev/tty.")
            return tty_stream
        except OSError:
            self.get_logger().warning("Could not open /dev/tty. Falling back to stdin.")
            return sys.stdin

    def _handle_key(self, character: str) -> None:
        now = time.monotonic()
        with self._lock:
            if character == "w":
                self._direction = 1
                self._last_drive_key_time = now
                return
            if character == "s":
                self._direction = -1
                self._last_drive_key_time = now
                return
            if character in ("+", "="):
                self.max_torque_nm = min(
                    self.torque_limit_nm,
                    self.max_torque_nm + self.torque_step_nm,
                )
                self.get_logger().info(
                    "Wheel max torque set to %.3f Nm." % self.max_torque_nm
                )
                return
            if character == "-":
                self.max_torque_nm = max(
                    0.0,
                    self.max_torque_nm - self.torque_step_nm,
                )
                self.get_logger().info(
                    "Wheel max torque set to %.3f Nm." % self.max_torque_nm
                )

    def _update_requested_torque(self) -> None:
        now = time.monotonic()
        with self._lock:
            if (
                self._last_drive_key_time is not None
                and now - self._last_drive_key_time > self.key_release_timeout_s
            ):
                self._direction = 0

            target_torque = self._direction * self.max_torque_nm
            dt = max(0.0, min(0.1, now - self._last_update_time))
            self._last_update_time = now

            ramp_reference_torque = max(
                abs(self._current_torque_nm),
                self.max_torque_nm,
                self.torque_step_nm,
            )
            max_delta = (ramp_reference_torque / self.ramp_time_s) * dt
            delta = max(-max_delta, min(max_delta, target_torque - self._current_torque_nm))
            self._current_torque_nm += delta

            if abs(target_torque) < 1.0e-4 and abs(self._current_torque_nm) < 1.0e-4:
                self._current_torque_nm = 0.0

            torque = self._current_torque_nm

        self._publish_torque(torque)

    def _publish_torque(self, torque_nm: float) -> None:
        msg = Float64()
        msg.data = float(torque_nm)
        self.requested_torque_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
