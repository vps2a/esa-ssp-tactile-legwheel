import argparse
from dataclasses import asdict
import threading
import time
import rclpy
import sys

from pathlib import Path
from rclpy.executors import SingleThreadedExecutor
from typing import Any

from legwheel_experiments.schemas import RunConfig, load_experiment_config, make_run_config, ExperimentConfig
from legwheel_experiments.state_machine import ExperimentStateMachine, ExperimentState
from legwheel_experiments.data_recorder import RunRecorder
from legwheel_experiments.rosbag_recorder import RosbagRecorder
from legwheel_experiments.experiment_runner_node import ExperimentRunnerNode

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

def collect_run_configuration() -> dict[str, Any]:
    print("\n=== Run Configuration ===")
    print("Please provide the following parameters for the experiment run:")

    run_length_loops = ask_float("Run length (in loops): ", minimum=0, maximum=2)

    print("[LEG PARAMETERS] Please provide the following parameters for the leg:")

    knee_stiffness_nm_per_rad = ask_float("Knee stiffness (in Nm/rad): ", minimum=0, maximum=50)
    knee_spring_zeroposition_rad = ask_float("Knee spring zero position (in rad): ", minimum=0, maximum=1)
    knee_damping_nms_per_rad = ask_float("Knee damping (in Nms/rad): ", minimum=0, maximum=5)

    commanded_wheel_torque_nm = ask_float("Commanded wheel torque (in Nm): ", minimum=0, maximum=8)
    wheel_torque_ramp_time_sec = ask_float("Wheel ramp time (in seconds): ", minimum=0.05, maximum=5)

    #print("[DATA ACQUISITION] Please provide the following parameters for data acquisition:")
    # camera_rate_hz = ask_float("Camera rate (in Hz): ", minimum=0, maximum=100)
    # leg_motor_telemetry_rate_hz = ask_float("Leg motor telemetry rate (in Hz): ", minimum=0, maximum=100)
    # wheel_motor_telemetry_rate_hz = ask_float("Wheel motor telemetry rate (in Hz): ", minimum=0, maximum=100)
    # encoder_data_rate_hz = ask_float("Encoder data rate (in Hz): ", minimum=0, maximum=100)

    return {
        "run_length_loops": run_length_loops,
        "leg_parameters": {
            "knee_stiffness_nm_per_rad": knee_stiffness_nm_per_rad,
            "knee_spring_zeroposition_rad": knee_spring_zeroposition_rad,
            "knee_damping_nms_per_rad": knee_damping_nms_per_rad,

        },
        "wheel_parameters": {
            "commanded_wheel_torque_nm": commanded_wheel_torque_nm,
            "wheel_torque_ramp_time_sec": wheel_torque_ramp_time_sec
        }
        # "data_acquisition_rates": {
        #     "camera_rate_hz": camera_rate_hz,
        #     "leg_motor_telemetry_rate_hz": leg_motor_telemetry_rate_hz,
        #     "wheel_motor_telemetry_rate_hz": wheel_motor_telemetry_rate_hz,
        #     "encoder_data_rate_hz": encoder_data_rate_hz
        # }
    }

def print_run_configuration(
    experiment_config: ExperimentConfig,
    run_config: RunConfig,
) -> None:
    """Display the validated values that will be saved and sent to the node."""
    print("\n=== Run Configuration Summary ===")
    print(f"Run length (in loops): {run_config.run_length_loops}")
    print("[LEG PARAMETERS]")
    for key, value in run_config.leg_parameters.items():
        print(f"  {key}: {value}")
    print("[WHEEL PARAMETERS]")
    for key, value in run_config.wheel_parameters.items():
        print(f"  {key}: {value}")

    #TODO Add more stuff in to complete the summary

    print("\n=== Experiment Configuration Summary ===")
    print(f"Experiment ID: {experiment_config.experiment_id}")
    print("[RIG CONFIG]") 
    for key, value in experiment_config.rig_config.items():
        print(f"  {key}: {value}")
    print("[CAMERA CONFIG]")
    print("  Camera Angle (in degrees): ", experiment_config.electronics_hardware["camera"]["camera_config"]["camera_front_angle_deg"])
    print("[ENVIRONMENT]")
    print("  Environment Description: ", experiment_config.environment["environment_description"])

def confirm_run_creation() -> bool:
    while True:
        confirmation = input("\nDo you want to create a new run with the above configuration? (y/n): ")
        if confirmation.lower() == "y":
            return True
        elif confirmation.lower() == "n":
            return False
        else:
            print("Invalid input. Please enter 'y' for yes or 'n' for no.")


def choose_run_number(recorder: RunRecorder) -> tuple[int, bool]:
    """Let the operator use the suggested number or explicitly overwrite a run."""
    suggested_number = recorder.find_next_run_number()

    while True:
        choice = input(
            f"The next available run number is {suggested_number}. "
            "Use it? (y/n): "
        ).strip().lower()

        if choice == "y":
            return suggested_number, False
        if choice != "n":
            print("Invalid input. Please enter 'y' or 'n'.")
            continue

        raw_number = input("Enter the existing run number to overwrite: ").strip()
        try:
            run_number = int(raw_number)
            target_directory = recorder.run_directory_for_number(run_number)
        except ValueError:
            print("Run number must be a positive integer.")
            continue

        if not target_directory.is_dir() or target_directory.is_symlink():
            print(f"No normal run directory exists at: {target_directory}")
            continue

        confirmation = input(
            f'All data in {target_directory} will be deleted. Type "OVERWRITE" '
            "to continue: "
        )
        if confirmation == "OVERWRITE":
            return run_number, True

        print("Overwrite cancelled. No files were deleted.")


def adjust_spring_zero_position(
    node: ExperimentRunnerNode,
    run_config: RunConfig,
) -> RunConfig:
    """Interactively adjust zero position using schema validation and a node ramp."""
    while True:
        choice = input(
            "Would you like to adjust the spring zero position? (y/n): "
        ).strip().lower()
        if choice == "n":
            return run_config
        if choice == "y":
            break
        print("Invalid input. Please enter 'y' or 'n'.")

    while True:
        current_zero_position = run_config.leg_parameters[
            "knee_spring_zeroposition_rad"
        ]
        response = input(
            "[SPRING ZEROPOS ADJUST] The current set spring zero position is "
            f"{current_zero_position}. Enter new value to set or type \"ready\" "
            "to confirm the current position: "
        ).strip()

        if response.lower() == "ready":
            return run_config

        try:
            requested_zero_position = float(response)
        except ValueError:
            print('Enter a number or type "ready".')
            continue

        # Rebuilding the config runs the same limits from schemas.py as initial input.
        raw_run_config = asdict(run_config)
        raw_run_config["leg_parameters"][
            "knee_spring_zeroposition_rad"
        ] = requested_zero_position
        try:
            updated_run_config = make_run_config(raw_run_config)
        except ValueError as error:
            print(f"Invalid spring zero position: {error}")
            continue

        # The node ramps to the new value instead of applying a step command.
        node.adjust_spring_zero_position(updated_run_config)
        run_config = updated_run_config
        print("Spring zero position updated gradually.")


def confirm_experiment_start() -> None:
    """Require an explicit final operator confirmation before motor motion begins."""
    while True:
        if input('Type "START" to begin the run: ') == "START":
            return
        print('Run has not started. Type "START" exactly to continue.')

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

    raw_run_config = collect_run_configuration()

    # Validate the operator input before showing confirmation or creating files.
    try:
        run_config = make_run_config(raw_run_config)
    except ValueError as error:
        print(f"Invalid run configuration: {error}")
        return

    print_run_configuration(experiment_config, run_config)

    if not confirm_run_creation():
        print("Run creation cancelled by the user.")
        return

    #Now creating the run recorder

    recorder = RunRecorder(experiment_directory)
    run_number, overwrite_existing = choose_run_number(recorder)

    run_directory = recorder.start_run(
        run_config=run_config,
        experiment_config=experiment_config,
        experiment_config_path=experiment_config_path,
        run_number=run_number,
        overwrite_existing=overwrite_existing,
    )
    print(f"Run directory created at: {run_directory}")

    # == Initialising ROS and starting the experiment runner node ==

    node = None
    executor = None
    executor_thread = None

    # Creating rosbag variables
    bag_recorder = None
    experiment_completed = False

    try:
        rclpy.init()
        node = ExperimentRunnerNode()
        # The node only passes this gate after it validates both configurations.
        node.set_configuration_valid(experiment_config, run_config)
        # node.set_recording_started()   

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

        bag_recorder = RosbagRecorder(run_directory)
        bag_directory = bag_recorder.start()
        node.set_recording_started()
        print(f"[ROSBAG] Started rosbag recording in: {bag_directory}")

        print("The wheel torque command will remain zero.")

        try:
            input("The leg is about to move. Press Enter to start the 3 second ramp...\n")
        except EOFError:
            print("Interactive input was unavailable; aborting before motor motion.")
            return

        # Wheel torque remains at zero while the mechanism settles before adjustment.
        
        print("Waiting 3 seconds for the wheel to settle into position...")
        time.sleep(3.0)
        run_config = adjust_spring_zero_position(node, run_config)
        # Save only the operator-confirmed value, after every schema check has passed.
        recorder.update_run_config(run_config)
        confirm_experiment_start()

        node.run_experiment_after_arm(run_config, experiment_config)
        experiment_completed = True


    except KeyboardInterrupt:
        print("\nExperiment runner interrupted by user.")
    
    finally:
        if bag_recorder is not None:
            try:
                bag_recorder.stop()
                print("[ROSBAG] Rosbag recording stopped and finalized.")
            except Exception as error:
                print(f"[ROSBAG] Failed to finalize rosbag: {error}")

        if experiment_completed:
            recorder.mark_complete()
        elif recorder.has_active_run():
            recorder.mark_aborted("Experiment did not complete")

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
