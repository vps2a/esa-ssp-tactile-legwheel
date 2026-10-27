"""Read serial encoder counts using the experiment's single configuration."""

from dataclasses import asdict
import json
from pathlib import Path

import serial
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64, String

from legwheel_encoder.config import (
    CONFIG_TOPIC,
    TICKS_TOPIC,
    DriverTickCounter,
    EncoderConfig,
)


class RotationEncoderNode(Node):
    def __init__(self):
        super().__init__("rotation_encoder_node")
        # Settings are loaded once. Changing experiments requires restarting the
        # driver, so the saved configuration cannot drift during a recording.
        self.declare_parameter(
            "experiment_config", "", ParameterDescriptor(read_only=True)
        )
        config_path = self.get_parameter("experiment_config").value
        if not config_path:
            raise ValueError("experiment_config must name an experiment YAML file.")
        self.config = EncoderConfig.from_yaml(Path(config_path))
        self._tick_counter = DriverTickCounter(
            zero_on_start=self.config.zero_on_start,
            invert_direction=self.config.invert_direction,
        )
        self.serial = serial.Serial(
            self.config.port, self.config.baudrate, timeout=0.01
        )
        self.serial.reset_input_buffer()
        self._last_hardware_ticks = None
        self._last_published_ticks = None
        self._last_sample_time = None

        self.joint_state_pub = self.create_publisher(
            JointState, "/rotation_encoder/joint_state", qos_profile_sensor_data
        )
        self.raw_ticks_pub = self.create_publisher(
            Int64, TICKS_TOPIC, qos_profile_sensor_data
        )
        # Late subscribers, including the experiment runner, can verify the
        # actual settings without assuming the source YAML is still unchanged.
        self._config_publisher = self.create_publisher(
            String,
            CONFIG_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._config_publisher.publish(String(data=json.dumps(asdict(self.config))))
        self.timer = self.create_timer(
            1.0 / self.config.publish_rate_hz, self.timer_callback
        )
        self.get_logger().info(
            f"Reading {self.config.port} at {self.config.baudrate} baud; "
            f"{self.config.ticks_per_legwheel_revolution} ticks per revolution."
        )

    def timer_callback(self):
        line = self.serial.readline().decode("utf-8", errors="replace").strip()
        if not line:
            return
        try:
            hardware_ticks = int(line)
        except ValueError:
            self.get_logger().warning(f"Invalid encoder line: {line}")
            return

        # Keep the existing serial outlier rule. Rejected samples must not
        # change either the startup origin or the last accepted count.
        if (
            self._last_hardware_ticks is not None
            and abs(hardware_ticks - self._last_hardware_ticks) > 3
        ):
            self.get_logger().warning(
                f"Rejecting false tick value: {hardware_ticks}; "
                f"last accepted tick value: {self._last_hardware_ticks}"
            )
            return
        self._last_hardware_ticks = hardware_ticks
        published_ticks = self._tick_counter.update(hardware_ticks)
        angle_rad = self.config.calibration.motion_angle(published_ticks)
        now = self.get_clock().now()
        velocity_rad_s = 0.0
        if self._last_published_ticks is not None:
            elapsed_s = (now - self._last_sample_time).nanoseconds * 1e-9
            if elapsed_s > 0.0:
                velocity_rad_s = (
                    self.config.calibration.motion_angle(
                        published_ticks - self._last_published_ticks
                    )
                    / elapsed_s
                )
        self._last_published_ticks = published_ticks
        self._last_sample_time = now

        self.raw_ticks_pub.publish(Int64(data=published_ticks))
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ["rotation_joint"]
        joint_state.position = [angle_rad]
        joint_state.velocity = [velocity_rad_s]
        self.joint_state_pub.publish(joint_state)

    def destroy_node(self):
        self.serial.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RotationEncoderNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
