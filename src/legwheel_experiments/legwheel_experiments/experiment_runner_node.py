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

import queue
import threading

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

        # #TODO: DELETE these later
        # self.set_configuration_valid()
        # self.set_recording_started()

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
        if self._state_machine.state is not ExperimentState.ARMED:
            raise RuntimeError(
                "Cannot enable motors: experiment is not armed"
            )

        message = Bool()
        message.data = True
        self._motor_status_publisher.publish(message)
        self.get_logger().info("Motors enabled.")

    def run_experiment_after_arm(self, run_config: dict[str, any]) -> None:
        if self._state_machine.state is not ExperimentState.ARMED:
            raise RuntimeError(
                "Cannot run experiment: experiment is not armed"
            )

        self.get_logger().info("Running the experiment...")
        self._state_machine.transition_to(
            ExperimentState.RUNNING
        )
        # Changing the marker
        self._change_marker(ExperimentState.RUNNING.name)

        # Turning the motors on
        self.enable_motors()

        #Configuring the wheel to go down to the target position, stiffness and damping
        input("The leg is about to move to its desired position. Press Enter to continue...")
        self._configure_legwheel_stiffness_and_zeropos(run_config["stiffness"], run_config["damping"], run_config["zero_position"])

        #Executing the run

    def _configure_legwheel_stiffness_and_zeropos(self, stiffness: list[float], damping: list[float], zero_pos: list[float]) -> None:
        #The stiffness and zeroposition are adjusted over a ramp time with a duration of 3 seconds
        duration = 3 #seconds

        if self._stiffness_publisher.get_last_published() is None:
            last_stiffness = [0.0] * len(stiffness)
        else:
            last_stiffness = self._stiffness_publisher.get_last_published().data
        
        if self._damping_publisher.get_last_published() is None:
            last_damping = [0.0] * len(damping)
        else:
            last_damping = self._damping_publisher.get_last_published().data

        if self._zero_position_publisher.get_last_published() is None:
            last_zeropos = [0.0] * len(zero_pos)
        else:
            last_zeropos = self._zero_position_publisher.get_last_published().data

        #now iterating over the duration and publishing the intermediate values
        start_time = time.monotonic()
        while time.monotonic() - start_time < duration:
            elapsed = time.monotonic() - start_time
            ratio = elapsed / duration

            intermediate_stiffness = [
                last + (target - last) * ratio
                for last, target in zip(last_stiffness, stiffness)
            ]
            intermediate_damping = [
                last + (target - last) * ratio
                for last, target in zip(last_damping, damping)
            ]
            intermediate_zeropos = [
                last + (target - last) * ratio
                for last, target in zip(last_zeropos, zero_pos)
            ]

            #Publishing the intermediate values
            stiffness_message = Float64MultiArray()
            stiffness_message.data = intermediate_stiffness
            self._stiffness_publisher.publish(stiffness_message)

            damping_message = Float64MultiArray()
            damping_message.data = intermediate_damping
            self._damping_publisher.publish(damping_message)

            zeropos_message = Float64MultiArray()
            zeropos_message.data = intermediate_zeropos
            self._zero_position_publisher.publish(zeropos_message)

            #Sleeping for a short time before the next iteration
            time.sleep(0.05)

        


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