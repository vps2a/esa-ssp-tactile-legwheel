'''
This node is responsible for:
Open the Arduino serial port.
Read tick counts.
Convert ticks to radians.
Publish a ROS2 message.
'''

import math
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64


class RotationEncoderNode(Node):
    def __init__(self):
        super().__init__("rotation_encoder_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("ticks_per_revolution", 62.5)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("invert_direction", False)
        self.declare_parameter("zero_on_start", True)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.ticks_per_revolution = self.get_parameter("ticks_per_revolution").value
        self.invert_direction = self.get_parameter("invert_direction").value

        self.serial = serial.Serial(self.port, self.baudrate, timeout=0.01)
        self.get_logger().info(f"Opened serial port {self.port} at {self.baudrate} baud.")
        self.serial.reset_input_buffer()

        self.zero_ticks = None
        self.last_ticks = None
        self.last_time = None

        self.last_tick_value = None

        self.joint_state_pub = self.create_publisher(
            JointState,
            "/rotation_encoder/joint_state",
            qos_profile_sensor_data,
        )

        self.raw_ticks_pub = self.create_publisher(
            Int64,
            "/rotation_encoder/ticks",
            qos_profile_sensor_data,
        )

        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(f"Reading encoder from {self.port}")

    def timer_callback(self):
        line = self.serial.readline().decode("utf-8").strip()

        #Rejecting the false ticks as well
        ticks_initialised = False
        if not ticks_initialised:
            last_tick_value = 0

        if not line:
            return

        try:
            ticks = int(line)
            self.get_logger().debug(f"Received ticks: {ticks}")

        except ValueError:
            self.get_logger().warning(f"Invalid encoder line: {line}")
            return

        # Rejecting false tick values that are too far from the last accepted value (greater than 3 ticks difference)
        if self.last_tick_value is not None:
            if abs(ticks - self.last_tick_value) > 3:
                self.get_logger().warning(f"Rejecting false tick value: {ticks}; last accepted tick value: {self.last_tick_value}")
                return
            
        #Updating only after the reading has been accepted
        self.last_tick_value = ticks

        if self.zero_ticks is None:
            self.zero_ticks = ticks

        relative_ticks = ticks - self.zero_ticks

        if self.invert_direction:
            relative_ticks *= -1

        angle_rad = (
            2.0 * math.pi * relative_ticks / self.ticks_per_revolution
        )

        now = self.get_clock().now()

        velocity_rad_s = 0.0
        if self.last_ticks is not None and self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt > 0.0:
                delta_ticks = relative_ticks - self.last_ticks
                velocity_rad_s = (
                    2.0 * math.pi * delta_ticks
                    / self.ticks_per_revolution
                    / dt
                )

        self.last_ticks = relative_ticks
        self.last_time = now

        tick_msg = Int64()
        tick_msg.data = relative_ticks
        self.raw_ticks_pub.publish(tick_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = now.to_msg()
        joint_msg.name = ["rotation_joint"]
        joint_msg.position = [angle_rad]
        joint_msg.velocity = [velocity_rad_s]

        self.joint_state_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RotationEncoderNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()