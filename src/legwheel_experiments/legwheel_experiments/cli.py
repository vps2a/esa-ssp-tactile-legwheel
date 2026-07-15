import argparse
import threading
import rclpy

from pathlib import Path
from rclpy.executors import SingleThreadedExecutor

from legwheel_experiments.schemas import load_experiment_config
from legwheel_experiments.state_machine import ExperimentStateMachine, ExperimentState
from legwheel_experiments.data_recorder import RunRecorder
from legwheel_experiments.experiment_runner_node import (
    ExperimentRunnerNode,
)

#Helper functions

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description = "Create and run a legwheel experiment"
    )

    parser.add_argument(
        "-ec",
        "--experiment-config",
        type=Path,
        required=True,
        help="Path to the experiment configuration YAML file"
    )

    return parser.parse_args()

#A better way of handling numering inputs than 'input' - won't crash
def ask_float(prompt: str, minimum: float | None = None, maximum: float | None = None) -> float:
    while True:
        raw_value = input(prompt)

        try:
            value = float(raw_value)
        except ValueError:
            print("Invalid input. Please enter a valid number.")
            continue

        if minimum is not None and value < minimum:
            print(f"Value must be at least {minimum}. Please try again.")
            continue

        if maximum is not None and value > maximum:
            print(f"Value must be at most {maximum}. Please try again.")
            continue

        return value

def collect_run_configuration() -> dict:
    print("\n=== Run Configuration ===")
    print("Please provide the following parameters for the experiment run:")

    run_length_loops = ask_float("Run length (in loops): ", minimum=0, maximum=2)

    print("[LEG PARAMETERS] Please provide the following parameters for the leg:")

    knee_stiffness_nm_per_rad = ask_float("Knee stiffness (in Nm/rad): ", minimum=0, maximum=50)
    knee_spring_zeroposition_rad = ask_float("Knee spring zero position (in rad): ", minimum=0, maximum=1)
    knee_damping_nms_per_rad = ask_float("Knee damping (in Nms/rad): ", minimum=0, maximum=5)

    commanded_wheel_torque_nm = ask_float("Commanded wheel torque (in Nm): ", minimum=0, maximum=8)
    maximium_wheel_speed_rad_per_sec = ask_float("Maximum wheel speed (in rad/s): ", minimum=0, maximum=4)

    print("[DATA ACQUISITION] Please provide the following parameters for data acquisition:")
    
    #TODO: Create default values for those
    camera_rate_hz = ask_float("Camera rate (in Hz): ", minimum=0, maximum=100)
    leg_motor_telemetry_rate_hz = ask_float("Leg motor telemetry rate (in Hz): ", minimum=0, maximum=100)
    wheel_motor_telemetry_rate_hz = ask_float("Wheel motor telemetry rate (in Hz): ", minimum=0, maximum=100)
    encoder_data_rate_hz = ask_float("Encoder data rate (in Hz): ", minimum=0, maximum=100)

    return {
        "run_length_loops": run_length_loops,
        "leg_parameters": {
            "knee_stiffness_nm_per_rad": knee_stiffness_nm_per_rad,
            "knee_spring_zeroposition_rad": knee_spring_zeroposition_rad,
            "knee_damping_nms_per_rad": knee_damping_nms_per_rad,

        },
        "wheel_parameters": {
            "commanded_wheel_torque_nm": commanded_wheel_torque_nm,
            "maximium_wheel_speed_rad_per_sec": maximium_wheel_speed_rad_per_sec
        },
        "data_acquisition_rates": {
            "camera_rate_hz": camera_rate_hz,
            "leg_motor_telemetry_rate_hz": leg_motor_telemetry_rate_hz,
            "wheel_motor_telemetry_rate_hz": wheel_motor_telemetry_rate_hz,
            "encoder_data_rate_hz": encoder_data_rate_hz
        }
    }

def print_run_configuration(experiment_config_path: Path, run_config: dict) -> None:
    print("\n=== Run Configuration Summary ===")
    print(f"Experiment configuration file: {experiment_config_path}")
    print(f"Run length (in loops): {run_config['run_length_loops']}")
    print("[LEG PARAMETERS]")
    for key, value in run_config["leg_parameters"].items():
        print(f"  {key}: {value}")
    print("[WHEEL PARAMETERS]")
    for key, value in run_config["wheel_parameters"].items():
        print(f"  {key}: {value}")
    print("[DATA ACQUISITION RATES]")
    for key, value in run_config["data_acquisition_rates"].items():
        print(f"  {key}: {value}")

    #TODO Add more stuff in to complete the summary

def confirm_run_creation() -> bool:
    while True:
        confirmation = input("\nDo you want to create a new run with the above configuration? (y/n): ")
        if confirmation.lower() == "y":
            return True
        elif confirmation.lower() == "n":
            return False
        else:
            print("Invalid input. Please enter 'y' for yes or 'n' for no.")

def main():
    print("=== LEGWHEEL EXPERIMENT RUNNER ===")

    arguments = parse_arguments()

    experiment_config_path = arguments.experiment_config
    experiment_directory = experiment_config_path.parent

    try:
        experiment_config = load_experiment_config(experiment_config_path)
    except (FileNotFoundError, KeyError, ValueError) as error:
        print(f"Error loading experiment configuration: {error}")
        return
    
    print(f"Loaded experiment : {experiment_config.experiment_id}")

    run_config = collect_run_configuration()

    print_run_configuration(experiment_config_path, run_config)

    if not confirm_run_creation():
        print("Run creation cancelled by the user.")
        return

    #Now creating the run recorder

    recorder = RunRecorder(experiment_directory)


    run_directory = recorder.start_run(run_config = run_config, experiment_config_path = experiment_config_path)
    print(f"Run directory created at: {run_directory}")

    # == Initialising ROS and starting the experiment runner node ==

    node = None
    executor = None
    executor_thread = None

    try:
        rclpy.init()
        node = ExperimentRunnerNode()
        print("We're here!")
        #TODO: Create real verifications later
        node.set_configuration_valid()
        node.set_recording_started()   

        # == Threading ==
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        executor_thread = threading.Thread(
            target=executor.spin,
            name="experiment_runner_executor",
            daemon=True,
        )

        executor_thread.start()

        # == Waiting for preflight checks ==
        print("\nWaiting for preflight checks...")

        while rclpy.ok():
            if node.wait_for_preflight(timeout_s=5.0):
                break

        print(f"Preflight: {node.preflight_status()}")

        #Double-checking just in case
        if node.experiment_state() is not ExperimentState.WAITING_FOR_ARM:
            raise RuntimeError(
                "Preflight ended without reaching WAITING_FOR_ARM"
            )

        print()
        print("Preflight passed.")
        print("The motors are currently disabled.")
        input("Review the robot and test area before continuing. Press Enter to continue...")
        input("Ensure the leg is in vertical position. Press Enter to continue...")
        print()

        # == Arming the motors ==

        operator_input = input(
            "Type ARM exactly to accept arming, or anything else to cancel: "
        )

        if operator_input != "ARM":
            print("Arming cancelled. Motors remain disabled.")
            return
        
        request_submitted = node.submit_arm_request()

        if not request_submitted:
            print("An arm request is already pending.")
            return
        
        arm_result = node.wait_for_arm_result(
            timeout_s=2.0
        )

        if arm_result is None:
            print("Arm request timed out. Motors remain disabled.")
            return

        accepted, reason = arm_result
        print(reason)

        if not accepted:
            return

        if node.experiment_state() is not ExperimentState.ARMED:
            raise RuntimeError(
                "Arm request was accepted, but the state is not ARMED"
            )

        print("Logical ARMED state reached.")
        print("No motor-enable command has been published.")

    except KeyboardInterrupt:
        print("\nExperiment runner interrupted by user.")
    
    finally:
        if node is not None:
            node.enter_safe_mode()

        if executor is not None:
            executor.shutdown()

        if executor_thread is not None:
            executor_thread.join(timeout=2.0)

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

# ===== OLD DEBUGGING FUNCTIONS =====

def create_new_run_directory() -> Path:
    #ask the user via a ui where to save the data
    default_path = Path("/home/parallels/Documents/Repositories/esa-ssp-tactile-legwheel/data/legwheel_experiment_runs/experiment_1")
    #Ask the user if this is the correct path and if not ask to change it
    path_flag = input(f"Is this the correct path to save the data? {default_path} (y/n): ")
    if path_flag.lower() != "y":
        new_path = input("Please enter the new path: ")
        default_path = Path(new_path)

    run = RunRecorder(default_path)
    next_run_number = run.find_next_run_number()

    next_run_number_flage = input(f"The next run number is: {next_run_number}. Do you want to use this run number? (y/n): ")
    if next_run_number_flage.lower() != "y":
        new_run_number = input("Please enter the new run number: ")
        next_run_number = int(new_run_number)

    run_directory = run.create_run({"run_number": next_run_number})
    
    return run_directory

def sample_transition_run():
    state_machine = ExperimentStateMachine()

    # Transition through the states
    state_machine.transition_to(ExperimentState.PREFLIGHT)
    state_machine.transition_to(ExperimentState.WAITING_FOR_ARM)
    state_machine.transition_to(ExperimentState.ARMED)
    state_machine.transition_to(ExperimentState.RUNNING)
    state_machine.transition_to(ExperimentState.STOPPING)
    state_machine.transition_to(ExperimentState.COMPLETE)

def sample_abort_run():
    state_machine = ExperimentStateMachine()

    state_machine.transition_to(ExperimentState.PREFLIGHT)
    state_machine.abort("Kot my wybuchl")
    state_machine.transition_to(ExperimentState.SAFE)
    
if __name__ == '__main__':
    main()