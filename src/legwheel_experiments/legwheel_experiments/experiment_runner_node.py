from collections.abc import Sequence
from numbers import Real
from typing import Any

import queue
import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

from legwheel_experiments.state_machine import (
    ExperimentState,
    ExperimentStateMachine,
)

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

        #Initializing the preflight complete event
        #unset = preflight has not completed
        #set   = preflight completed
        self._preflight_complete_event = threading.Event()

        #Initializing the arm request and result queues
        #The two queues communicate in opposite directions:
        # CLI ──arm request──▶ ROS node
        # CLI ◀──result────── ROS node
        self._arm_request_queue: queue.Queue[None] = queue.Queue(maxsize=1)
        self._arm_request_completed_event = threading.Event()

        self._arm_result_queue: queue.Queue[tuple[bool, str]] = queue.Queue(
            maxsize=1
        )


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

        self._last_commanded_stiffness = [0.0, 0.0]
        self._last_commanded_damping = [0.0, 0.0]
        self._last_commanded_zero_position = [0.0, 0.0]

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

        # === Entering preflight ===

        self._state_machine.transition_to(
            ExperimentState.PREFLIGHT
        )
        
        self._preflight_timer = self.create_timer(
            0.1,
            self._preflight_callback,
        )

        #Adding also an arm request timer
        self._arm_request_timer = self.create_timer(
            0.1,
            self._process_arm_request,
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
    
    def _publish_zero_wheel_torque(self) -> None:
        torque_message = Float64()
        torque_message.data = 0.0
        self._wheel_torque_publisher.publish(torque_message)

    def disable_motors(self) -> None:
        message = Bool()
        message.data = False
        self._motor_status_publisher.publish(message)

    def enter_safe_mode(self) -> None:
    #This is the first function that is gonna be published after startup to make sure the motors are disabled
        self._publish_zero_wheel_torque()

        self.disable_motors()

    # == Arming processing functions ==

    def _process_arm_request(self) -> None:
        if self._state_machine.state not in (
            ExperimentState.WAITING_FOR_ARM,
            ExperimentState.ARMED,
        ):
            return

        # Continue enforcing disabled commands in both states.
        self.enter_safe_mode()

        try:
            self._arm_request_queue.get_nowait()
        except queue.Empty:
            return
        if self._state_machine.state is not ExperimentState.WAITING_FOR_ARM:
                self._store_arm_result(
                    accepted=False,
                    reason="Arm request rejected: experiment is not waiting for arm",
                )
                return

        failure_reason = self._get_preflight_failure_reason()

        if failure_reason is not None:
            self._store_arm_result(
                accepted=False,
                reason=f"Arm request rejected: {failure_reason}",
            )
            return

        self._state_machine.transition_to(
            ExperimentState.ARMED
        )

        self._store_arm_result(
            accepted=True,
            reason="Arm request accepted; motors remain disabled",
        )

        self._arm_request_completed_event.set()

    def _store_arm_result(
        self,
        accepted: bool,
        reason: str,
    ) -> None:
        try:
            self._arm_result_queue.put_nowait(
                (accepted, reason)
            )
        except queue.Full:
            self.get_logger().error(
                "Could not store arm result: result queue is full"
            )

        if accepted:
            self.get_logger().info(reason)
        else:
            self.get_logger().warning(reason)


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

        failure_reason = self._get_preflight_failure_reason()

        if failure_reason is not None:
            self._set_preflight_status(failure_reason)
            return

        self._set_preflight_status("Preflight passed")

        self._state_machine.transition_to(
            ExperimentState.WAITING_FOR_ARM
        )

        self._preflight_complete_event.set()

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

    def wait_for_preflight(self, timeout_s: float | None = None) -> bool:
        return self._preflight_complete_event.wait(timeout=timeout_s)
    
    def submit_arm_request(self) -> bool:
        try:
            self._arm_request_queue.put_nowait(None)
        except queue.Full:
            return False
        return True
        
    def wait_for_arm_result(
        self,
        timeout_s: float | None = None,
    ) -> tuple[bool, str] | None:
        try:
            return self._arm_result_queue.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def preflight_status(self) -> str:
        return self._preflight_status

    def experiment_state(self) -> ExperimentState:
        return self._state_machine.state
    
    # === Running the experiment ===

    def enable_motors(self) -> None:
        if self._state_machine.state not in (
            ExperimentState.ARMED,
            ExperimentState.RUNNING,
        ):
            raise RuntimeError(
                "Cannot enable motors: experiment is not armed or running"
            )

        message = Bool()
        message.data = True
        self._motor_status_publisher.publish(message)
        self.get_logger().info("Motors enabled.")

    def run_experiment_after_arm(self, run_config: dict[str, Any]) -> None:
        if self._state_machine.state is not ExperimentState.ARMED:
            raise RuntimeError(
                "Cannot run experiment: experiment is not armed"
            )

        leg_parameters = run_config["leg_parameters"]
        knee_stiffness = leg_parameters["knee_stiffness_nm_per_rad"]
        knee_damping = leg_parameters["knee_damping_nms_per_rad"]
        knee_zero_position = leg_parameters["knee_spring_zeroposition_rad"]

        if not all(
            self._is_numeric(value)
            for value in (knee_stiffness, knee_damping, knee_zero_position)
        ):
            raise ValueError(
                "Knee stiffness, damping, and zero position must be numeric"
            )

        # Current CLI collects knee-only scalar values. Keep joint 0 passive
        # and apply the requested knee parameters to joint 1 for now.
        stiffness = [0.0, float(knee_stiffness)]
        damping = [0.0, float(knee_damping)]
        zero_position = [0.0, float(knee_zero_position)]

        self._publish_zero_wheel_torque()


        self.enable_motors()

        self._configure_legwheel_stiffness_and_zeropos(
            stiffness=stiffness,
            damping=damping,
            zero_position=zero_position,
        )

        self.get_logger().info("Post-arm leg parameter ramp completed.")

    def _configure_legwheel_stiffness_and_zeropos(
        self,
        stiffness: Sequence[float],
        damping: Sequence[float],
        zero_position: Sequence[float],
    ) -> None:
        stiffness_target = self._validate_two_numeric_values(
            "stiffness",
            stiffness,
        )
        damping_target = self._validate_two_numeric_values(
            "damping",
            damping,
        )
        zero_position_target = self._validate_two_numeric_values(
            "zero_position",
            zero_position,
        )

        #The stiffness and zeroposition are adjusted over a ramp time with a duration of 3 seconds
        duration_s = 3.0
        publish_period_s = 0.05

        last_stiffness = self._last_commanded_stiffness
        last_damping = self._last_commanded_damping
        last_zero_position = self._current_position_from_encoder()

        #now iterating over the duration and publishing the intermediate values
        start_time = time.monotonic()
        while time.monotonic() - start_time < duration_s:
            elapsed = time.monotonic() - start_time
            ratio = elapsed / duration_s

            intermediate_stiffness = [
                last + (target - last) * ratio
                for last, target in zip(last_stiffness, stiffness_target)
            ]
            intermediate_damping = [
                last + (target - last) * ratio
                for last, target in zip(last_damping, damping_target)
            ]
            intermediate_zeropos = [
                last + (target - last) * ratio
                for last, target in zip(last_zero_position, zero_position_target)
            ]

            self._publish_leg_parameters(
                stiffness=intermediate_stiffness,
                damping=intermediate_damping,
                zero_position=intermediate_zeropos,
            )

            #Sleeping for a short time before the next iteration
            time.sleep(publish_period_s)

        self._publish_leg_parameters(
            stiffness=stiffness_target,
            damping=damping_target,
            zero_position=zero_position_target,
        )

    def _current_position_from_encoder(self) -> list[float]:
        if self._latest_leg_state is None:
            raise RuntimeError("Cannot read leg position: no leg telemetry")

        if self.leg_state_age_s() > self._maximum_state_age_s:
            raise RuntimeError("Cannot rely on leg position data: leg telemetry is stale")

        return self._validate_two_numeric_values(
            "leg encoder position",
            self._latest_leg_state.position,
        )

    def _publish_leg_parameters(
        self,
        stiffness: list[float],
        damping: list[float],
        zero_position: list[float],
    ) -> None:
        stiffness_message = Float64MultiArray()
        stiffness_message.data = stiffness
        self._stiffness_publisher.publish(stiffness_message)

        damping_message = Float64MultiArray()
        damping_message.data = damping
        self._damping_publisher.publish(damping_message)

        zero_position_message = Float64MultiArray()
        zero_position_message.data = zero_position
        self._zero_position_publisher.publish(zero_position_message)

        self._last_commanded_stiffness = list(stiffness)
        self._last_commanded_damping = list(damping)
        self._last_commanded_zero_position = list(zero_position)

    @staticmethod
    def _validate_two_numeric_values(
        name: str,
        values: Sequence[float],
    ) -> list[float]:
        if (
            isinstance(values, (str, bytes))
            or not isinstance(values, Sequence)
            or len(values) != 2
        ):
            raise ValueError(f"{name} must contain exactly 2 numeric values")

        if not all(ExperimentRunnerNode._is_numeric(value) for value in values):
            raise ValueError(f"{name} must contain exactly 2 numeric values")

        return [float(value) for value in values]

    @staticmethod
    def _is_numeric(value: Any) -> bool:
        return isinstance(value, Real) and not isinstance(value, bool)

        


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
