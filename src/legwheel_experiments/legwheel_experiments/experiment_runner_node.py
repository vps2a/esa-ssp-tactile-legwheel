from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from legwheel_experiments.state_machine import (
    ExperimentState,
    ExperimentStateMachine,
)

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class ExperimentRunnerNode(Node):
    def __init__(self):
        super().__init__("experiment_runner")

        #Initialising the state machine
        self._state_machine = ExperimentStateMachine()

        #Initialising the legwheel states
        #These store the ROS messages
        self._latest_leg_state = None
        self._latest_wheel_state = None
        #These store when the node received them
        self._leg_state_received_at_s = None
        self._wheel_state_received_at_s = None

        self._motor_status_publisher = self.create_publisher(
            Bool,
            "/legwheel/motor_status",
            10,
        )

        self._zero_position_publisher = self.create_publisher(
            Float64MultiArray,
            "/legwheel/spring_zero_position",
            10,
        )

        self._stiffness_publisher = self.create_publisher(
            Float64MultiArray,
            "/legwheel/spring_constant",
            10,
        )

        self._damping_publisher = self.create_publisher(
            Float64MultiArray,
            "/legwheel/damping_constant",
            10,
        )

        self._wheel_torque_publisher = self.create_publisher(
            Float64,
            "/wheel/requested_torque",
            10,
        )

        self._leg_state_subscription = self.create_subscription(
            JointState,
            "/legwheel/joint_states",
            self._leg_state_callback,
            qos_profile_sensor_data,
        )

        self._wheel_state_subscription = self.create_subscription(
            JointState,
            "/wheel/wheel_state",
            self._wheel_state_callback,
            qos_profile_sensor_data,
        )

        self._maximum_state_age_s = 0.5 #telemetry sample older than 0.5 seconds is considered stale

        self._configuration_valid = False #means the CLI successfully loaded and validated the experiment YAML.
        self._recording_started = False #means the CLI successfully loaded and validated the experiment YAML.

        self._preflight_status = "Preflight has not started"

        #Entering preflight
        self._state_machine.transition_to(
            ExperimentState.PREFLIGHT
        )
        self._preflight_timer = self.create_timer(
            0.1,
            self._preflight_callback,
        )

        self.get_logger().info("Experiment Runner Node initialized.")

        self.enter_safe_mode()

        self.get_logger().info("Motors disabled and safe mode entered.")

    # == Telemetry Callbacks ==

    def _leg_state_callback(self, message: JointState) -> None:
        self._latest_leg_state = message
        self._leg_state_received_at_s = time.monotonic()

    def _wheel_state_callback(self, message: JointState) -> None:
        self._latest_wheel_state = message
        self._wheel_state_received_at_s = time.monotonic()

    # == Telemetry helper functions and inspection ==
    def has_leg_state(self) -> bool:
        return self._latest_leg_state is not None
    
    def has_wheel_state(self) -> bool:
        return self._latest_wheel_state is not None
    
        #Calculate telemetry age
        #If no message has arrived, the age is treated as infinity - this is useful for checking if the telemetry is stale
    def leg_state_age_s(self) -> float:
        if self._leg_state_received_at_s is None:
            return float("inf")

        return time.monotonic() - self._leg_state_received_at_s
    
    def wheel_state_age_s(self) -> float:
        if self._wheel_state_received_at_s is None:
            return float("inf")

        return time.monotonic() - self._wheel_state_received_at_s

    # == Safe publishing ==
    
    def disable_motors(self) -> None:
        message = Bool()
        message.data = False
        self._motor_status_publisher.publish(message)

    def enter_safe_mode(self) -> None:
    #This is the first function that is gonna be published after startup to make sure the motors are disabled
        torque_message = Float64()
        torque_message.data = 0.0
        self._wheel_torque_publisher.publish(torque_message)

        self.disable_motors()

    # == CLI readiness notifications ==

    #Helper functions to give CLI a methode to change the configuration and recording status
    def set_configuration_valid(self) -> None:
        self._configuration_valid = True

    def set_recording_started(self) -> None:
        self._recording_started = True

    # == Preflight ==

    #Preflight status helper functions
    def _preflight_callback(self) -> None:
        if self._state_machine.state is not ExperimentState.PREFLIGHT:
            return

        self.enter_safe_mode()

        #TODO: DELETE these later
        self.set_configuration_valid()
        self.set_recording_started()

        failure_reason = self._get_preflight_failure_reason()

        if failure_reason is not None:
            self._set_preflight_status(failure_reason)
            return

        self._set_preflight_status("Preflight passed")

        self._state_machine.transition_to(
            ExperimentState.WAITING_FOR_ARM
        )

    def _set_preflight_status(self, status: str) -> None: #Publish status changes only when they change
        if status == self._preflight_status:
            return

        self._preflight_status = status
        self.get_logger().info(status)
    
    def _get_preflight_failure_reason(self) -> str | None:
        if not self._configuration_valid:
            return "Waiting for valid experiment configuration"

        if not self._recording_started:
            return "Waiting for recording to start"

        if not self.has_leg_state():
            return "Waiting for leg telemetry"

        if not self.has_wheel_state():
            return "Waiting for wheel telemetry"

        if self.leg_state_age_s() > self._maximum_state_age_s:
            return "Leg telemetry is stale"

        if self.wheel_state_age_s() > self._maximum_state_age_s:
            return "Wheel telemetry is stale"

        if self._motor_status_publisher.get_subscription_count() == 0:
            return "Waiting for a subscriber to /legwheel/motor_status"

        if self._wheel_torque_publisher.get_subscription_count() == 0:
            return "Waiting for a subscriber to /wheel/requested_torque"

        return None

    # == Read-only public info ==

    def preflight_status(self) -> str:
        return self._preflight_status

    def experiment_state(self) -> ExperimentState:
        return self._state_machine.state


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunnerNode()

    try:
        node.enter_safe_mode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.enter_safe_mode()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()