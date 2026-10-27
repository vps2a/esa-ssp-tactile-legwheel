"""Publish one shared IMU/encoder/wheel pose stream for every robot visualizer.

Sensor callbacks only collect measurements. A steady-clock timer runs the
shared estimator on a bounded recent history, then publishes supported samples
with their original acquisition times. This works with live sensors and bag
playback without introducing a second pitch algorithm or any knee fallback.
"""

import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int64

from legwheel_encoder.config import TICKS_TOPIC, EncoderCalibration, read_experiment

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.encoder_replay import RecordedEncoderTicks
from legwheel_kinematics.imu_pitch_buffer import ImuPitchBuffer
from legwheel_kinematics.pose_history import KINEMATIC_STATE_TOPIC, stamp_ns


def _sensor_queue() -> QoSProfile:
    """Keep native samples while a batch fit briefly occupies the executor.

    The default sensor queue holds only five messages. A fit or a burst of
    replayed messages can exceed that; losing those samples creates false gaps.
    Best-effort reliability remains compatible with the camera driver.
    """
    return QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)


class KinematicJointStateNode(Node):
    """Share IMU pitch and wheel-refined encoder yaw with every visualizer."""

    def __init__(self) -> None:
        super().__init__("kinematic_joint_state")
        self.declare_parameter("experiment_config", "")
        self.declare_parameter("imu_topic", "/legwheel_rgbd/gyro_accel/sample")
        self.declare_parameter("encoder_topic", TICKS_TOPIC)
        self.declare_parameter("encoder_bag", "")
        self.declare_parameter("wheel_topic", "/wheel/wheel_state")
        self.declare_parameter("history_window_s", 10.0)
        self.declare_parameter("maximum_imu_gap_s", 0.05)
        self.declare_parameter("maximum_wheel_gap_s", 0.05)
        self.declare_parameter("maximum_pitch_uncertainty_deg", 1.0)
        self.declare_parameter("estimate_rate_hz", 5.0)
        self.declare_parameter("smoothing_delay_s", 0.6)

        config_path = self.get_parameter("experiment_config").value
        if not config_path:
            raise ValueError("experiment_config must name an experiment YAML file.")
        experiment = read_experiment(Path(config_path))
        geometry = KinematicsConfig.from_experiment_dict(experiment)
        self._encoder_calibration = EncoderCalibration.from_experiment_dict(
            experiment
        )
        self._using_sim_time = bool(self.get_parameter("use_sim_time").value)
        self._recorded_encoder = None
        self._last_loaded_encoder_time_ns = None
        if self._using_sim_time:
            bag_argument = self.get_parameter("encoder_bag").value
            bag_directory = (
                Path(bag_argument).expanduser()
                if bag_argument
                else Path(config_path).parent / "rosbag"
            )
            if not bag_directory.is_dir():
                raise ValueError(
                    f"Cannot find encoder playback bag at {bag_directory}. "
                    "Set encoder_bag to this recording's rosbag directory."
                )
            self._recorded_encoder = RecordedEncoderTicks.from_bag(
                bag_directory,
                self._encoder_calibration,
                self.get_parameter("encoder_topic").value,
            )
        history_window_s = float(self.get_parameter("history_window_s").value)
        estimate_rate_hz = float(self.get_parameter("estimate_rate_hz").value)
        self._smoothing_delay_s = float(self.get_parameter("smoothing_delay_s").value)
        if not math.isfinite(estimate_rate_hz) or estimate_rate_hz <= 0.0:
            raise ValueError("estimate_rate_hz must be finite and positive.")
        # Circular acceleration uses a centered encoder fit with 0.6 seconds
        # of lookahead. Publishing its interior avoids noisy endpoint fits. The
        # modest delay also lets IMU and image messages arrive in either order.
        if (
            not math.isfinite(self._smoothing_delay_s)
            or self._smoothing_delay_s < 0.6
            or history_window_s <= self._smoothing_delay_s + 0.5
        ):
            raise ValueError(
                "Require smoothing_delay_s >= 0.6 and a history window at least "
                "0.5 seconds longer than that delay."
            )
        self._buffer = ImuPitchBuffer(
            geometry,
            window_s=history_window_s,
            maximum_gap_s=float(self.get_parameter("maximum_imu_gap_s").value),
            maximum_uncertainty_deg=float(
                self.get_parameter("maximum_pitch_uncertainty_deg").value
            ),
            maximum_wheel_gap_s=float(self.get_parameter("maximum_wheel_gap_s").value),
            encoder_origin_motion_rad=(
                0.0 if self._recorded_encoder is not None else None
            ),
        )
        self._history_window_ns = int(history_window_s * 1e9)
        self._last_clock_ns = None
        self._last_estimated_time_ns = None
        self._last_published_time_ns = None

        # Internal pose delivery is reliable: dropping a single invalid-state
        # barrier could otherwise let an overlay interpolate through an impact.
        # This is also compatible with robot_state_publisher's sensor-data QoS.
        self._publisher = self.create_publisher(
            JointState, KINEMATIC_STATE_TOPIC, QoSProfile(depth=200)
        )
        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self._imu_callback,
            _sensor_queue(),
        )
        if self._recorded_encoder is None:
            self.create_subscription(
                Int64,
                self.get_parameter("encoder_topic").value,
                self._encoder_callback,
                _sensor_queue(),
            )
        self.create_subscription(
            JointState,
            self.get_parameter("wheel_topic").value,
            self._wheel_callback,
            _sensor_queue(),
        )
        self.create_subscription(
            Clock, "/clock", self._clock_callback, qos_profile_sensor_data
        )
        # Bound CPU work per wall second even during accelerated bag playback.
        # Acquisition timestamps, not this timer, determine every output pose.
        self.create_timer(
            1.0 / estimate_rate_hz,
            self._publish_estimate,
            clock=RosClock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            "Waiting for camera IMU, central encoder and wheel velocity; all "
            "visualizers will use shared IMU pitch and wheel-refined encoder yaw."
        )

    def _belongs_to_current_playback(self, timestamp_ns: int) -> bool:
        """Reject queued pre-seek measurements from the future of a rewound bag.

        Ordinary camera timestamp jitter is allowed. A full second of positive
        lead tolerates camera/clock delivery order, but an old DDS message must
        not move a freshly cleared buffer back into the previous playback epoch.
        """
        return not (
            self._using_sim_time
            and self._last_clock_ns is not None
            and timestamp_ns > self._last_clock_ns + 1_000_000_000
        )

    def _imu_callback(self, message: Imu) -> None:
        timestamp_ns = stamp_ns(message.header.stamp)
        if not self._belongs_to_current_playback(timestamp_ns):
            return
        try:
            self._buffer.add_imu(
                timestamp_ns,
                (
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ),
                (
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ),
                frame_id=message.header.frame_id,
            )
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)

    def _encoder_callback(self, message: Int64) -> None:
        """Timestamp live receipt only; playback reads exact times from the bag."""
        self._buffer.add_encoder(
            self.get_clock().now().nanoseconds,
            self._encoder_calibration.motion_angle(message.data),
        )

    def _clock_callback(self, message: Clock) -> None:
        """A seek starts a new history; small camera timestamp jitter does not."""
        if not self._using_sim_time:
            return
        clock_ns = stamp_ns(message.clock)
        if self._last_clock_ns is not None and (
            clock_ns < self._last_clock_ns
            or clock_ns - self._last_clock_ns > self._history_window_ns
        ):
            self._buffer.clear()
            self._last_estimated_time_ns = None
            self._last_published_time_ns = None
            self._last_loaded_encoder_time_ns = None
        self._last_clock_ns = clock_ns
        # /clock controls which samples are available, never their timestamps.
        # After a seek, refill the visible window using the same recording zero.
        if self._recorded_encoder is not None:
            first_ns = clock_ns - self._history_window_ns
            if self._last_loaded_encoder_time_ns is not None:
                first_ns = max(first_ns, self._last_loaded_encoder_time_ns + 1)
            samples = self._recorded_encoder.window(first_ns, clock_ns)
            for timestamp_ns, angle_rad in samples:
                self._buffer.add_encoder(int(timestamp_ns), float(angle_rad))
                self._last_loaded_encoder_time_ns = int(timestamp_ns)

    def _wheel_callback(self, message: JointState) -> None:
        """Use measured wheel velocity, never motor position or commanded speed."""
        timestamp_ns = stamp_ns(message.header.stamp)
        if not self._belongs_to_current_playback(timestamp_ns):
            return
        try:
            index = list(message.name).index("wheel_joint")
            self._buffer.add_wheel(timestamp_ns, float(message.velocity[index]))
        except (ValueError, IndexError) as error:
            self.get_logger().warning(
                f"Cannot use wheel odometry: {error}", throttle_duration_sec=5.0
            )

    def _publish_estimate(self) -> None:
        latest_time_ns = self._buffer.latest_time_ns
        if latest_time_ns is None or latest_time_ns == self._last_estimated_time_ns:
            return
        try:
            trajectory = self._buffer.estimate()
        except ValueError as error:
            # Bad startup data (for example only nonfinite encoder positions)
            # must leave the node waiting, so later good measurements recover.
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        if trajectory is None:
            return
        self._last_estimated_time_ns = latest_time_ns
        cutoff_ns = latest_time_ns - int(self._smoothing_delay_s * 1e9)
        # Publish each acquisition time once. Revising previously emitted poses
        # would make TF and image annotations depend on message arrival order.
        new_samples = trajectory.times_ns <= cutoff_ns
        if self._last_published_time_ns is not None:
            new_samples &= trajectory.times_ns > self._last_published_time_ns
        available_samples = trajectory.available
        for index in np.flatnonzero(new_samples):
            timestamp_ns = int(trajectory.times_ns[index])
            output = JointState()
            output.header.stamp.sec = timestamp_ns // 1_000_000_000
            output.header.stamp.nanosec = timestamp_ns % 1_000_000_000
            output.header.frame_id = "legwheel_base"
            if available_samples[index]:
                try:
                    yaw_rad = self._buffer.yaw_at(timestamp_ns)
                except ValueError:
                    pass
                else:
                    output.name = ["theta_y_joint", "theta_p_joint"]
                    output.position = [yaw_rad, float(trajectory.theta_p_rad[index])]
            # An empty JointState marks an unsupported timestamp. Custom
            # visualizers retain it as an interpolation barrier; the standard
            # robot_state_publisher has no joint update to turn into invalid TF.
            # Low-confidence estimates are ordinary poses here. Empty updates
            # are reserved for timestamps where no estimate actually exists.
            self._publisher.publish(output)
            self._last_published_time_ns = timestamp_ns


def main(args=None) -> None:
    """Run the shared IMU-derived kinematic JointState publisher."""
    rclpy.init(args=args)
    node = KinematicJointStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
