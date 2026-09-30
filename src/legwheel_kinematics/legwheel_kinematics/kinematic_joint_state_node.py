"""Publish the two joints consumed by the LegWheel URDF frame tree."""

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.transforms import theta_p_from_config


class KinematicJointStateNode(Node):
    """Convert the measured knee angle into notebook theta_y/theta_p joints."""

    def __init__(self) -> None:
        super().__init__("kinematic_joint_state")
        self.declare_parameter("experiment_config", "")
        config_path = self.get_parameter("experiment_config").value
        if not config_path:
            raise ValueError("experiment_config must name an experiment YAML file.")
        self._config = KinematicsConfig.from_yaml(Path(config_path))
        self._publisher = self.create_publisher(
            JointState,
            "/legwheel/kinematic_joint_states",
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            "/legwheel/joint_states",
            self._leg_state_callback,
            qos_profile_sensor_data,
        )

    def _leg_state_callback(self, message: JointState) -> None:
        try:
            knee_index = list(message.name).index("knee_joint")
            knee_joint = float(message.position[knee_index])
            theta_p = theta_p_from_config(knee_joint, self._config)
        except (ValueError, IndexError) as error:
            self.get_logger().error(f"Cannot publish kinematic state: {error}")
            return

        output = JointState()
        output.header = message.header
        output.name = ["theta_y_joint", "theta_p_joint"]
        # TODO: theta_y is consciously fixed at zero while the processed track
        # is radially circular. Replace this with measured theta_y when the
        # frame tree needs world-fixed or non-circular geometry.
        theta_y = 0.0
        output.position = [theta_y, theta_p]
        self._publisher.publish(output)


def main(args=None) -> None:
    """Run the derived kinematic JointState publisher."""
    rclpy.init(args=args)
    node = KinematicJointStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
