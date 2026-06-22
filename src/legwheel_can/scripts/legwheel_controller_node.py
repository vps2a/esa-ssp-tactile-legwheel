#!/usr/bin/env python3

import math
import select
import sys
import threading
from typing import List, TextIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String


HIP_INDEX = 0
KNEE_INDEX = 1


def make_array_msg(values: List[float]) -> Float64MultiArray:
    msg = Float64MultiArray()
    msg.data = list(values)
    return msg


class LegwheelControllerNode(Node):
    """Terminal command publisher for the legwheel impedance node."""

    def __init__(self) -> None:
        super().__init__("legwheel_controller_node")

        self.declare_parameter("initial_hip_zero_position_rad", 0.0)
        self.declare_parameter("initial_knee_zero_position_rad", 0.0)
        self.declare_parameter("initial_hip_spring_constant", 1.0)
        self.declare_parameter("initial_knee_spring_constant", 1.0)
        self.declare_parameter("initial_hip_damping_constant", 0.05)
        self.declare_parameter("initial_knee_damping_constant", 0.05)
        self.declare_parameter("publish_initial_commands", True)
        self.declare_parameter("auto_enable", False)

        self.spring_zero_position = [
            self.get_parameter("initial_hip_zero_position_rad").value,
            self.get_parameter("initial_knee_zero_position_rad").value,
        ]
        self.spring_constant = [
            self.get_parameter("initial_hip_spring_constant").value,
            self.get_parameter("initial_knee_spring_constant").value,
        ]
        self.damping_constant = [
            self.get_parameter("initial_hip_damping_constant").value,
            self.get_parameter("initial_knee_damping_constant").value,
        ]
        self.publish_initial_commands = self.get_parameter("publish_initial_commands").value
        self.auto_enable = self.get_parameter("auto_enable").value

        self._validate_initial_values()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._startup_publish_count = 0
        self._startup_enable_published = False

        self.motor_status_pub = self.create_publisher(
            Bool, "/legwheel/motor_status", 10
        )
        self.spring_zero_pub = self.create_publisher(
            Float64MultiArray, "/legwheel/spring_zero_position", 10
        )
        self.spring_constant_pub = self.create_publisher(
            Float64MultiArray, "/legwheel/spring_constant", 10
        )
        self.damping_constant_pub = self.create_publisher(
            Float64MultiArray, "/legwheel/damping_constant", 10
        )
        self.command_sub = self.create_subscription(
            String,
            "/legwheel/controller_command",
            self._handle_command_msg,
            10,
        )

        self.startup_timer = self.create_timer(0.5, self._publish_startup_commands)

        self.get_logger().info(
            "Legwheel controller ready. Type commands followed by Enter. Type 'help' for commands."
        )
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._input_thread.is_alive():
            self._input_thread.join(timeout=1.0)

    def _validate_initial_values(self) -> None:
        for value in self.spring_zero_position:
            if not math.isfinite(value):
                raise ValueError("Initial spring zero positions must be finite.")
        for value in self.spring_constant:
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("Initial spring constants must be finite and non-negative.")
        for value in self.damping_constant:
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("Initial damping constants must be finite and non-negative.")

    def _publish_startup_commands(self) -> None:
        if self.publish_initial_commands:
            self._publish_all_command_arrays()
            if self._startup_publish_count == 0:
                self.get_logger().info(
                    "Publishing initial spring zero, spring constant, and damping constant commands."
                )

        if self.auto_enable and not self._startup_enable_published:
            self._publish_motor_status(True)
            self._startup_enable_published = True
            self.get_logger().warning(
                "auto_enable is true: published /legwheel/motor_status=true."
            )
        elif not self.auto_enable and self._startup_publish_count == 0:
            self.get_logger().info(
                "Motors are not enabled by the controller on startup. Type 'e' when ready."
            )

        self._startup_publish_count += 1
        if self._startup_publish_count >= 5:
            self.startup_timer.cancel()

    def _input_loop(self) -> None:
        input_stream = self._open_command_input()
        close_stream = input_stream is not sys.stdin

        try:
            while not self._stop_event.is_set():
                readable, _, _ = select.select([input_stream], [], [], 0.1)
                if not readable:
                    continue

                line = input_stream.readline()
                if line == "":
                    self.get_logger().warning(
                        "Controller input closed. Use /legwheel/controller_command for commands."
                    )
                    return

                self._handle_command(line.strip())
        finally:
            if close_stream:
                input_stream.close()

    def _open_command_input(self) -> TextIO:
        try:
            tty = open("/dev/tty", "r", encoding="utf-8")
            self.get_logger().info("Reading controller commands from /dev/tty.")
            return tty
        except OSError:
            self.get_logger().warning(
                "Could not open /dev/tty. Falling back to stdin and /legwheel/controller_command."
            )
            return sys.stdin

    def _handle_command_msg(self, msg: String) -> None:
        self._handle_command(msg.data)

    def _handle_command(self, line: str) -> None:
        tokens = line.split()
        if not tokens:
            return

        command = tokens[0].lower()
        if command in ("s", "stop", "disable"):
            self._publish_motor_status(False)
            self.get_logger().error("Published motor_status=false.")
            return

        if command in ("e", "enable", "start"):
            self._publish_motor_status(True)
            self.get_logger().warning("Published motor_status=true.")
            return

        if command in ("help", "h", "?"):
            self._print_help()
            return

        if command == "status":
            self._print_status()
            return

        if len(tokens) != 3:
            self.get_logger().error(
                "Invalid command. Expected '<hip|knee> <set_zeropos|set_spring|set_damp> <value>', 'e', or 's'."
            )
            return

        joint_index = self._parse_joint(tokens[0])
        if joint_index is None:
            self.get_logger().error(
                "Unknown joint '%s'. Use 'hip' or 'knee'." % tokens[0]
            )
            return

        try:
            value = float(tokens[2])
        except ValueError:
            self.get_logger().error("Invalid numeric value '%s'." % tokens[2])
            return

        if not math.isfinite(value):
            self.get_logger().error("Value must be finite.")
            return

        action = tokens[1].lower()
        if action in ("set_zeropos", "set_zero", "set_zero_position"):
            self._set_spring_zero(joint_index, value)
            return

        if action == "set_spring":
            if value < 0.0:
                self.get_logger().error("Spring constant must be non-negative.")
                return
            self._set_spring_constant(joint_index, value)
            return

        if action in ("set_damp", "set_damping"):
            if value < 0.0:
                self.get_logger().error("Damping constant must be non-negative.")
                return
            self._set_damping_constant(joint_index, value)
            return

        self.get_logger().error(
            "Unknown action '%s'. Use set_zeropos, set_spring, or set_damp."
            % tokens[1]
        )

    def _set_spring_zero(self, joint_index: int, value: float) -> None:
        with self._lock:
            self.spring_zero_position[joint_index] = value
            self.spring_zero_pub.publish(make_array_msg(self.spring_zero_position))
        self.get_logger().info(
            "Published %s spring zero position %.4f rad."
            % (self._joint_name(joint_index), value)
        )

    def _set_spring_constant(self, joint_index: int, value: float) -> None:
        with self._lock:
            self.spring_constant[joint_index] = value
            self.spring_constant_pub.publish(make_array_msg(self.spring_constant))
        self.get_logger().info(
            "Published %s spring constant %.4f Nm/rad."
            % (self._joint_name(joint_index), value)
        )

    def _set_damping_constant(self, joint_index: int, value: float) -> None:
        with self._lock:
            self.damping_constant[joint_index] = value
            self.damping_constant_pub.publish(make_array_msg(self.damping_constant))
        self.get_logger().info(
            "Published %s damping constant %.4f N/(rad/s)."
            % (self._joint_name(joint_index), value)
        )

    def _publish_all_command_arrays(self) -> None:
        with self._lock:
            self.spring_zero_pub.publish(make_array_msg(self.spring_zero_position))
            self.spring_constant_pub.publish(make_array_msg(self.spring_constant))
            self.damping_constant_pub.publish(make_array_msg(self.damping_constant))

    def _publish_motor_status(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.motor_status_pub.publish(msg)

    def _print_help(self) -> None:
        self.get_logger().info(
            "Commands: 'e' or 'enable' allow motors; 's' disables motors; "
            "'hip set_zeropos 0.0'; 'knee set_spring 4.0'; 'hip set_damp 5.0'; 'status'."
        )

    def _print_status(self) -> None:
        with self._lock:
            self.get_logger().info(
                "Current commands: zero=[%.4f, %.4f] rad, spring=[%.4f, %.4f] Nm/rad, "
                "damping=[%.4f, %.4f] N/(rad/s)."
                % (
                    self.spring_zero_position[HIP_INDEX],
                    self.spring_zero_position[KNEE_INDEX],
                    self.spring_constant[HIP_INDEX],
                    self.spring_constant[KNEE_INDEX],
                    self.damping_constant[HIP_INDEX],
                    self.damping_constant[KNEE_INDEX],
                )
            )

    @staticmethod
    def _parse_joint(token: str):
        joint = token.lower()
        if joint == "hip":
            return HIP_INDEX
        if joint == "knee":
            return KNEE_INDEX
        return None

    @staticmethod
    def _joint_name(joint_index: int) -> str:
        return "hip" if joint_index == HIP_INDEX else "knee"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LegwheelControllerNode()
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
