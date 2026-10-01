"""Stage 1 packet isolation and Stage 2 feature extraction."""

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

import numpy as np

from legwheel_dataset.bag_reader import (
    DEPTH_IMAGE_TOPIC,
    RGB_IMAGE_TOPIC,
    BagIndex,
    extract_selected_images,
    index_bag,
)
from legwheel_dataset.config import ProcessingConfig
from legwheel_dataset.features import (
    calculate_spectra,
    fit_plane_ransac,
    slope_from_plane,
)
from legwheel_dataset.images import (
    decode_image,
    depth_point_map_in_base,
    masked_crop,
    polygon_mask,
)
from legwheel_dataset.patch_geometry import (
    project_full_track_boundaries,
    project_track_patch,
)
from legwheel_dataset.run_analysis import (
    CorrectedThetaTrajectory,
    build_corrected_theta_trajectory,
    effective_rate_statistics,
    find_steady_torque_period,
    time_weighted_mean_absolute_speed,
)
from legwheel_dataset.synchronization import (
    interpolate_scalar,
    pair_image_timestamps,
    resample_channels,
)
from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.projection import CameraCalibration
from legwheel_kinematics.transforms import theta_p_from_config


# Only measurements acquired near 100 Hz belong on the common ML telemetry
# grid. Central-encoder ticks remain a motion reference for theta_y correction,
# but are deliberately excluded from packet arrays and spectral features.
RESAMPLED_TELEMETRY_STREAM_NAMES = ("leg", "wheel", "imu")
RESAMPLED_TELEMETRY_CHANNEL_COUNT = 15


def _load_yaml(path: Path) -> dict[str, Any]:
    import yaml

    with path.open("r", encoding="utf-8") as file:
        values = yaml.safe_load(file)
    if not isinstance(values, dict):
        raise ValueError(f"Configuration must contain a mapping: {path}")
    return values


def _find_unique(run_directory: Path, pattern: str, description: str) -> Path:
    candidates = sorted(run_directory.glob(pattern))
    if len(candidates) != 1:
        raise ValueError(
            f"Run directory must contain exactly one {description}; "
            f"found {len(candidates)}."
        )
    return candidates[0]


def find_experiment_snapshot(run_directory: Path) -> Path:
    """Find the unique experiment snapshot saved by RunRecorder."""
    return _find_unique(
        run_directory,
        "experiment_config_run_*_snapshot.yaml",
        "experiment configuration snapshot",
    )


def _load_run_config(run_directory: Path) -> dict[str, Any]:
    return _load_yaml(run_directory / "run_config.yaml")


def _validate_registered_depth(run_directory: Path) -> dict[str, Any]:
    """Require the simple, calibrated color-aligned depth mode agreed for Stage 2."""
    camera_path = _find_unique(
        run_directory,
        "camera_driver_config_run_*_snapshot.yaml",
        "camera-driver configuration snapshot",
    )
    camera_config = _load_yaml(camera_path)
    if camera_config.get("depth_registration") is not True:
        raise ValueError("camera_depth_registration_must_be_true")
    if str(camera_config.get("align_target_stream", "")).upper() != "COLOR":
        raise ValueError("camera_align_target_stream_must_be_COLOR")
    return camera_config


def _dataset_identifier(
    experiment_config: dict[str, Any],
    run_config: dict[str, Any],
    processing_config: ProcessingConfig,
) -> str:
    payload = json.dumps(
        {
            "experiment": experiment_config,
            "run": run_config,
            "processing": processing_config.to_dict(),
        },
        sort_keys=True,
        separators=(",", ":"),
        default=str,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()[:12]


def _finite_column(
    times: np.ndarray,
    values: np.ndarray,
    column: int,
) -> tuple[np.ndarray, np.ndarray]:
    finite = np.isfinite(values[:, column])
    return times[finite], values[finite, column]


def _resample_all_streams(
    bag_index: BagIndex,
    centre_time_ns: int,
    window_ms: float,
    config: ProcessingConfig,
    valid_start_time_ns: int,
    valid_end_time_ns: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    """Resample the 15 ML channels without adding encoder or ramp data."""
    sampled_streams = []
    sampled_masks = []
    channel_names = []
    relative_times_ns = None
    for stream_name in RESAMPLED_TELEMETRY_STREAM_NAMES:
        stream = bag_index.streams[stream_name]
        source_times_ns, source_values = stream.arrays()
        inside = (
            (source_times_ns >= valid_start_time_ns)
            & (source_times_ns <= valid_end_time_ns)
        )
        source_times_ns = source_times_ns[inside]
        source_values = source_values[inside]
        if source_times_ns.size < 2:
            raise ValueError(f"telemetry_stream_{stream_name}_has_too_few_samples")
        values, valid, stream_relative_times_ns = resample_channels(
            source_times_ns,
            source_values,
            centre_time_ns,
            window_ms,
            config.telemetry_resample_hz,
            config.maximum_interpolation_gap_ms,
        )
        if relative_times_ns is None:
            relative_times_ns = stream_relative_times_ns
        elif not np.array_equal(relative_times_ns, stream_relative_times_ns):
            raise RuntimeError("Telemetry streams produced different sample grids.")
        sampled_streams.append(values)
        sampled_masks.append(valid)
        channel_names.extend(stream.channel_names)
    if len(channel_names) != RESAMPLED_TELEMETRY_CHANNEL_COUNT:
        raise RuntimeError(
            f"Expected {RESAMPLED_TELEMETRY_CHANNEL_COUNT} telemetry channels, "
            f"received {len(channel_names)}."
        )
    return (
        np.column_stack(sampled_streams),
        np.column_stack(sampled_masks),
        relative_times_ns,
        channel_names,
    )


def _window_has_complete_timestamp_coverage(
    bag_index: BagIndex,
    start_time_ns: int,
    end_time_ns: int,
    maximum_gap_ns: int,
) -> bool:
    """Check local-window timestamp coverage before doing any interpolation."""
    for stream_name in RESAMPLED_TELEMETRY_STREAM_NAMES:
        times_ns, _ = bag_index.streams[stream_name].arrays()
        if (
            times_ns.size < 2
            or start_time_ns < times_ns[0]
            or end_time_ns > times_ns[-1]
        ):
            return False
        left = max(0, int(np.searchsorted(times_ns, start_time_ns)) - 1)
        right = min(times_ns.size, int(np.searchsorted(times_ns, end_time_ns)) + 1)
        if np.any(np.diff(times_ns[left:right]) > maximum_gap_ns):
            return False
    return True


def _normalise_rgb(image: np.ndarray, encoding: str) -> tuple[np.ndarray, str]:
    if encoding == "bgr8":
        return image[:, :, ::-1], "rgb8"
    if encoding == "bgra8":
        return image[:, :, [2, 1, 0, 3]], "rgba8"
    return image, encoding


def _image_writer(path: Path, kind: str):
    def write(message) -> None:
        image = decode_image(message)
        if kind == "rgb":
            image, _ = _normalise_rgb(image, str(message.encoding))
        np.save(path, image, allow_pickle=False)

    return write


def _camera_inputs(
    image_metadata,
    calibrations,
    timestamp_ns: int,
) -> CameraCalibration:
    calibration = calibrations.at(timestamp_ns)
    camera_info_frame = calibrations.frame_id_at(timestamp_ns)
    if image_metadata.frame_id and camera_info_frame:
        if image_metadata.frame_id != camera_info_frame:
            raise ValueError("image_and_camera_info_frame_ids_differ")
    if (
        calibration.width != image_metadata.width
        or calibration.height != image_metadata.height
    ):
        raise ValueError("image_and_camera_info_dimensions_differ")
    return calibration


def _safe_identifier(value: Any) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9]+", "", str(value))
    return cleaned or "unknown"


def _run_number(run_directory: Path) -> str:
    match = re.search(r"run_(\d+)$", run_directory.name)
    return match.group(1) if match else _safe_identifier(run_directory.name)


def _theta_filename_value(theta_y_rad: float) -> str:
    rendered = f"{abs(theta_y_rad):.3f}".replace(".", "p")
    return ("m" if theta_y_rad < 0.0 else "") + rendered


def _packet_identifier(
    experiment_id: str,
    run_number: str,
    theta_y_rad: float,
    rgb_time_ns: int,
    depth_time_ns: int,
    dataset_identifier: str,
) -> str:
    hash_payload = (
        f"{experiment_id}|{run_number}|{rgb_time_ns}|{depth_time_ns}|"
        f"{dataset_identifier}"
    ).encode("utf-8")
    packet_hash = hashlib.sha256(hash_payload).hexdigest()[:8]
    return (
        f"e{_safe_identifier(experiment_id)}_r{run_number}_"
        f"theta_{_theta_filename_value(theta_y_rad)}_{packet_hash}"
    )


def _run_motion_analysis(
    bag_index: BagIndex,
    experiment_config: dict[str, Any],
    run_config: dict[str, Any],
    kinematics_config: KinematicsConfig,
    processing_config: ProcessingConfig,
) -> tuple[CorrectedThetaTrajectory, float, dict[str, Any]]:
    """Calculate the accepted run, corrected theta_y, local duration and rates."""
    try:
        target_torque_nm = float(
            run_config["wheel_parameters"]["commanded_wheel_torque_nm"]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("run_config_missing_commanded_wheel_torque_nm") from error

    torque_times, torque_values = bag_index.streams["torque_command"].arrays()
    torque_period = find_steady_torque_period(
        torque_times,
        torque_values[:, 0],
        target_torque_nm,
        processing_config.minimum_target_torque_fraction,
    )

    try:
        ticks_per_revolution = float(
            experiment_config["electronics_hardware"]["encoder_setup"]
            ["ticks_per_legwheel_revolution"]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "experiment_config_missing_ticks_per_legwheel_revolution"
        ) from error
    if ticks_per_revolution <= 0.0:
        raise ValueError("ticks_per_legwheel_revolution_must_be_positive")
    encoder_times, encoder_tick_values = bag_index.streams["ticks"].arrays()
    encoder_times, encoder_ticks = _finite_column(
        encoder_times,
        encoder_tick_values,
        0,
    )
    # /rotation_encoder/ticks already includes the driver's configured direction.
    # Converting the actual count difference also preserves multi-tick updates.
    encoder_angles = encoder_ticks * (2.0 * math.pi / ticks_per_revolution)
    wheel_times, wheel_values = bag_index.streams["wheel"].arrays()
    wheel_velocity_column = bag_index.streams["wheel"].channel_names.index(
        "wheel_joint.velocity"
    )
    wheel_times, wheel_velocity = _finite_column(
        wheel_times,
        wheel_values,
        wheel_velocity_column,
    )

    track_radius_m = (
        kinematics_config.Beam_radius - kinematics_config.Wheel_width / 2.0
    )
    trajectory = build_corrected_theta_trajectory(
        encoder_times,
        encoder_angles,
        wheel_times,
        wheel_velocity,
        torque_period,
        kinematics_config.wheel_radius_mm / 1000.0,
        processing_config.grouser_effective_height_m,
        track_radius_m,
        processing_config.maximum_interpolation_gap_ms,
    )

    omega_avg_rad_s = time_weighted_mean_absolute_speed(
        trajectory.wheel_times_ns,
        trajectory.wheel_velocity_rad_s,
        trajectory.start_time_ns,
        trajectory.end_time_ns,
        trajectory.maximum_gap_ns,
    )
    duration_s = (trajectory.end_time_ns - trajectory.start_time_ns) * 1e-9
    average_rig_angular_speed_rad_s = abs(
        float(trajectory.theta_y_at_ticks_rad[-1]) / duration_s
    )
    if average_rig_angular_speed_rad_s <= 0.0:
        raise ValueError("average_corrected_rig_speed_is_zero")
    local_window_ms = (
        processing_config.alpha_sp_rad
        / average_rig_angular_speed_rad_s
        * 1000.0
    )

    source_rates = {}
    rate_limit_violations = []
    for stream_name in RESAMPLED_TELEMETRY_STREAM_NAMES:
        stream_times, _ = bag_index.streams[stream_name].arrays()
        statistics = effective_rate_statistics(
            stream_times,
            trajectory.start_time_ns,
            trajectory.end_time_ns,
        )
        source_rates[stream_name] = statistics
        median_rate = statistics["median_rate_hz"]
        if median_rate is None or processing_config.telemetry_resample_hz > (
            float(median_rate)
            * (1.0 + processing_config.source_rate_tolerance_fraction)
        ):
            rate_limit_violations.append(stream_name)

    steady_wheel = trajectory.wheel_velocity_rad_s[
        (trajectory.wheel_times_ns >= trajectory.start_time_ns)
        & (trajectory.wheel_times_ns <= trajectory.end_time_ns)
    ]
    report = {
        "status": "analysed",
        "torque_period": {
            "threshold_start_time_ns": torque_period.start_time_ns,
            "threshold_end_time_ns": torque_period.end_time_ns,
            "configured_target_nm": torque_period.configured_target_nm,
            "minimum_target_fraction": (
                processing_config.minimum_target_torque_fraction
            ),
            "acceptance_threshold_nm": torque_period.acceptance_threshold_nm,
            "held_command_at_start_nm": torque_period.held_command_at_start_nm,
        },
        "corrected_motion": {
            "t0_ns": trajectory.start_time_ns,
            "end_time_ns": trajectory.end_time_ns,
            "encoder_angle_at_t0_rad": float(
                trajectory.raw_encoder_angles_rad[0]
            ),
            "theta_y_end_rad": float(trajectory.theta_y_at_ticks_rad[-1]),
            "travel_direction": trajectory.travel_direction,
            "complete_encoder_interval_count": int(
                trajectory.correction_factors.size
            ),
            "rejected_reverse_transition_count": (
                trajectory.rejected_reverse_transition_count
            ),
            "track_radius_m": track_radius_m,
            "wheel_to_track_radius_ratio": trajectory.radius_ratio,
            "correction_factor_min": float(np.min(trajectory.correction_factors)),
            "correction_factor_max": float(np.max(trajectory.correction_factors)),
            "correction_factor_mean": float(np.mean(trajectory.correction_factors)),
        },
        "wheel_speed": {
            "time_weighted_mean_absolute_rad_s": omega_avg_rad_s,
            "minimum_absolute_rad_s": float(np.min(np.abs(steady_wheel))),
            "maximum_absolute_rad_s": float(np.max(np.abs(steady_wheel))),
            "median_absolute_rad_s": float(np.median(np.abs(steady_wheel))),
            "standard_deviation_absolute_rad_s": float(
                np.std(np.abs(steady_wheel))
            ),
            "sample_count": int(steady_wheel.size),
        },
        "local_window": {
            "derivation": "alpha_sp_rad / average_corrected_rig_angular_speed",
            "alpha_sp_rad": processing_config.alpha_sp_rad,
            "average_corrected_rig_angular_speed_rad_s": (
                average_rig_angular_speed_rad_s
            ),
            "local_window_ms": local_window_ms,
        },
        "measured_effective_rates": source_rates,
        "motion_reference_effective_rate": {
            "stream": "rotation_encoder_ticks",
            **effective_rate_statistics(
                encoder_times,
                trajectory.start_time_ns,
                trajectory.end_time_ns,
            ),
        },
        "resampled_telemetry_streams": list(
            RESAMPLED_TELEMETRY_STREAM_NAMES
        ),
        "resampled_telemetry_channel_count": (
            RESAMPLED_TELEMETRY_CHANNEL_COUNT
        ),
        "excluded_from_packet_telemetry": [
            "rotation_joint.position",
            "rotation_joint.velocity",
            "rotation_encoder.ticks",
        ],
        "requested_resample_rate_hz": processing_config.telemetry_resample_hz,
        "source_rate_tolerance_fraction": (
            processing_config.source_rate_tolerance_fraction
        ),
        "rate_limit_violations": rate_limit_violations,
        "experiment_encoder_setup": experiment_config.get(
            "electronics_hardware", {}
        ).get("encoder_setup", {}),
    }
    return trajectory, local_window_ms, report


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, default=str), encoding="utf-8")


def validate_processing_configuration(
    run_directory: Path,
    processing_config: ProcessingConfig,
    experiment_config_path: Path | None = None,
) -> dict[str, Any]:
    """Analyse a run and report consequences without creating a dataset."""
    run_directory = run_directory.resolve()
    processing_config.validate()
    if experiment_config_path is None:
        experiment_config_path = find_experiment_snapshot(run_directory)
    experiment_config = _load_yaml(experiment_config_path.resolve())
    run_config = _load_run_config(run_directory)
    _validate_registered_depth(run_directory)
    kinematics_config = KinematicsConfig.from_experiment_dict(experiment_config)
    bag_index = index_bag(run_directory / "rosbag")
    trajectory, local_window_ms, report = _run_motion_analysis(
        bag_index,
        experiment_config,
        run_config,
        kinematics_config,
        processing_config,
    )

    spectral_sample_count = max(
        2,
        int(round(
            processing_config.spectral_window_ms
            * processing_config.telemetry_resample_hz
            / 1000.0
        )) + 1,
    )
    report["spectral_analysis"] = {
        "spectral_window_ms": processing_config.spectral_window_ms,
        "sample_count": spectral_sample_count,
        "nyquist_frequency_hz": processing_config.telemetry_resample_hz / 2.0,
        "fft_bin_spacing_hz": (
            processing_config.telemetry_resample_hz / spectral_sample_count
        ),
    }
    report["spectral_window_not_shorter_than_local_window"] = bool(
        processing_config.spectral_window_ms >= local_window_ms
    )

    rgb_times = np.asarray(
        [image.timestamp_ns for image in bag_index.rgb_images], dtype=np.int64
    )
    depth_times = np.asarray(
        [image.timestamp_ns for image in bag_index.depth_images], dtype=np.int64
    )
    pairs = pair_image_timestamps(
        rgb_times,
        depth_times,
        int(round(processing_config.rgb_depth_max_delta_ms * 1_000_000.0)),
    )
    leg_times, leg_values = bag_index.streams["leg"].arrays()
    knee_column = bag_index.streams["leg"].channel_names.index(
        "knee_joint.position"
    )
    knee_times, knee_values = _finite_column(
        leg_times, leg_values, knee_column
    )
    depth_pixel_counts = []
    for pair in pairs:
        if not trajectory.start_time_ns <= pair.image_time_ns <= trajectory.end_time_ns:
            continue
        try:
            theta_y_image = trajectory.theta_y_at(pair.image_time_ns)
            theta_y_depth = trajectory.theta_y_at(pair.depth_time_ns)
            patch_centre = (
                theta_y_image
                + trajectory.travel_direction * processing_config.alpha_off_rad
            )
            depth_metadata = bag_index.depth_images[pair.depth_index]
            calibration = _camera_inputs(
                depth_metadata,
                bag_index.depth_calibrations,
                pair.depth_time_ns,
            )
            knee_joint = interpolate_scalar(
                knee_times, knee_values, pair.depth_time_ns
            )
            projection = project_track_patch(
                kinematics_config,
                theta_y_depth,
                theta_p_from_config(knee_joint, kinematics_config),
                patch_centre,
                processing_config.alpha_sp_rad,
                calibration,
                processing_config.track_point_count,
            )
            if not projection.fully_visible:
                continue
            mask = polygon_mask(
                depth_metadata.height,
                depth_metadata.width,
                projection.polygon_pixels,
            )
            depth_pixel_counts.append(int(np.count_nonzero(mask)))
            if len(depth_pixel_counts) >= 20:
                break
        except ValueError:
            continue
    report["estimated_depth_patch_pixels"] = {
        "sampled_fully_visible_packet_count": len(depth_pixel_counts),
        "minimum": min(depth_pixel_counts) if depth_pixel_counts else None,
        "median": (
            float(np.median(depth_pixel_counts)) if depth_pixel_counts else None
        ),
        "maximum": max(depth_pixel_counts) if depth_pixel_counts else None,
    }
    report["configuration_valid"] = bool(
        not report["rate_limit_violations"]
        and report["spectral_window_not_shorter_than_local_window"]
        and depth_pixel_counts
    )
    return report


def isolate_packets(
    run_directory: Path,
    processing_config: ProcessingConfig,
    experiment_config_path: Path | None = None,
    output_directory: Path | None = None,
) -> Path:
    """Convert synchronized MCAP data into self-contained Stage 1 packets."""
    run_directory = run_directory.resolve()
    processing_config.validate()
    if experiment_config_path is None:
        experiment_config_path = find_experiment_snapshot(run_directory)
    experiment_config_path = experiment_config_path.resolve()
    experiment_config = _load_yaml(experiment_config_path)
    run_config = _load_run_config(run_directory)
    camera_driver_config = _validate_registered_depth(run_directory)
    kinematics_config = KinematicsConfig.from_experiment_dict(experiment_config)
    bag_directory = run_directory / "rosbag"
    if not bag_directory.is_dir():
        raise FileNotFoundError(f"Rosbag directory does not exist: {bag_directory}")

    dataset_identifier = _dataset_identifier(
        experiment_config,
        run_config,
        processing_config,
    )
    if output_directory is None:
        output_directory = run_directory / "derived" / dataset_identifier
    output_directory = output_directory.resolve()
    if output_directory.exists():
        raise FileExistsError(
            "Derived output already exists; raw results are never overwritten: "
            f"{output_directory}"
        )
    packets_directory = output_directory / "packets"
    packets_directory.mkdir(parents=True)

    bag_index = index_bag(bag_directory)
    trajectory, local_window_ms, analysis_report = _run_motion_analysis(
        bag_index,
        experiment_config,
        run_config,
        kinematics_config,
        processing_config,
    )
    analysis_report.update(
        {
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "dataset_identifier": dataset_identifier,
            "rejected_candidates": [],
            "rejection_counts": {},
        }
    )
    analysis_report_path = packets_directory / "analysis_report.json"
    _write_json(analysis_report_path, analysis_report)

    if analysis_report["rate_limit_violations"]:
        analysis_report["status"] = "failed_source_rate_validation"
        _write_json(analysis_report_path, analysis_report)
        names = ", ".join(analysis_report["rate_limit_violations"])
        raise ValueError(
            "telemetry_resample_hz exceeds the measured effective source "
            f"rate (including tolerance) for: {names}. See {analysis_report_path}"
        )
    if processing_config.spectral_window_ms < local_window_ms:
        analysis_report["status"] = "failed_window_validation"
        _write_json(analysis_report_path, analysis_report)
        raise ValueError(
            "spectral_window_ms is shorter than the calculated local window: "
            f"{local_window_ms:.3f} ms. See {analysis_report_path}"
        )

    np.savez_compressed(
        packets_directory / "run_motion_analysis.npz",
        tick_times_ns=trajectory.tick_times_ns,
        raw_encoder_angles_rad=trajectory.raw_encoder_angles_rad,
        theta_y_at_ticks_rad=trajectory.theta_y_at_ticks_rad,
        correction_factors=trajectory.correction_factors,
        predicted_interval_angles_rad=trajectory.predicted_interval_angles_rad,
        encoder_interval_angles_rad=trajectory.encoder_interval_angles_rad,
    )

    rgb_times_ns = np.asarray(
        [image.timestamp_ns for image in bag_index.rgb_images], dtype=np.int64
    )
    depth_times_ns = np.asarray(
        [image.timestamp_ns for image in bag_index.depth_images], dtype=np.int64
    )
    synchronized_pairs = pair_image_timestamps(
        rgb_times_ns,
        depth_times_ns,
        int(round(processing_config.rgb_depth_max_delta_ms * 1_000_000.0)),
    )

    leg_times_ns, leg_values = bag_index.streams["leg"].arrays()
    knee_column = bag_index.streams["leg"].channel_names.index(
        "knee_joint.position"
    )
    knee_times_ns, knee_values = _finite_column(
        leg_times_ns,
        leg_values,
        knee_column,
    )
    if knee_times_ns.size < 2:
        raise ValueError("bag_contains_too_few_valid_knee_joint_samples")

    manifest_records = []
    image_requests = {}
    last_saved_image_time_ns = None
    minimum_packet_spacing_ns = int(round(
        1_000_000_000.0 / processing_config.maximum_packet_saving_frequency_hz
    ))
    maximum_gap_ns = int(round(
        processing_config.maximum_interpolation_gap_ms * 1_000_000.0
    ))
    experiment_id = str(experiment_config.get("experiment_id", "unknown"))
    run_number = _run_number(run_directory)

    def reject(pair, reason: str) -> None:
        counts = analysis_report["rejection_counts"]
        counts[reason] = counts.get(reason, 0) + 1
        analysis_report["rejected_candidates"].append(
            {
                "rgb_time_ns": pair.rgb_time_ns,
                "depth_time_ns": pair.depth_time_ns,
                "image_time_ns": pair.image_time_ns,
                "reason": reason,
            }
        )

    for pair in synchronized_pairs:
        if pair.image_time_ns < trajectory.start_time_ns:
            reject(pair, "image_before_t0")
            continue
        if pair.image_time_ns > trajectory.end_time_ns:
            reject(pair, "image_after_corrected_run_end")
            continue
        if (
            last_saved_image_time_ns is not None
            and pair.image_time_ns - last_saved_image_time_ns
            < minimum_packet_spacing_ns
        ):
            reject(pair, "maximum_packet_saving_frequency")
            continue

        try:
            theta_y_image = trajectory.theta_y_at(pair.image_time_ns)
            theta_y_rgb = trajectory.theta_y_at(pair.rgb_time_ns)
            theta_y_depth = trajectory.theta_y_at(pair.depth_time_ns)
            patch_centre_angle_rad = (
                theta_y_image
                + trajectory.travel_direction * processing_config.alpha_off_rad
            )
            contact_time_ns, angular_residual_rad = (
                trajectory.first_future_crossing_time(
                    pair.image_time_ns,
                    processing_config.alpha_off_rad,
                )
            )

            rgb_metadata = bag_index.rgb_images[pair.rgb_index]
            depth_metadata = bag_index.depth_images[pair.depth_index]
            rgb_calibration = _camera_inputs(
                rgb_metadata,
                bag_index.rgb_calibrations,
                pair.rgb_time_ns,
            )
            depth_calibration = _camera_inputs(
                depth_metadata,
                bag_index.depth_calibrations,
                pair.depth_time_ns,
            )
            knee_joint_rgb = interpolate_scalar(
                knee_times_ns, knee_values, pair.rgb_time_ns
            )
            knee_joint_depth = interpolate_scalar(
                knee_times_ns, knee_values, pair.depth_time_ns
            )
            theta_p_rgb = theta_p_from_config(knee_joint_rgb, kinematics_config)
            theta_p_depth = theta_p_from_config(
                knee_joint_depth, kinematics_config
            )

            rgb_projection = project_track_patch(
                kinematics_config,
                theta_y_rgb,
                theta_p_rgb,
                patch_centre_angle_rad,
                processing_config.alpha_sp_rad,
                rgb_calibration,
                processing_config.track_point_count,
            )
            depth_projection = project_track_patch(
                kinematics_config,
                theta_y_depth,
                theta_p_depth,
                patch_centre_angle_rad,
                processing_config.alpha_sp_rad,
                depth_calibration,
                processing_config.track_point_count,
            )
            if not rgb_projection.fully_visible:
                raise ValueError("rgb_patch_not_fully_visible")
            if not depth_projection.fully_visible:
                raise ValueError("depth_patch_not_fully_visible")

            rgb_mask = polygon_mask(
                rgb_metadata.height,
                rgb_metadata.width,
                rgb_projection.polygon_pixels,
            )
            depth_mask = polygon_mask(
                depth_metadata.height,
                depth_metadata.width,
                depth_projection.polygon_pixels,
            )
            if not np.any(rgb_mask):
                raise ValueError("rgb_patch_mask_is_empty")
            if not np.any(depth_mask):
                raise ValueError("depth_patch_mask_is_empty")

            half_local_window_ns = int(round(local_window_ms * 500_000.0))
            local_start_ns = contact_time_ns - half_local_window_ns
            local_end_ns = contact_time_ns + half_local_window_ns
            if (
                local_start_ns < trajectory.start_time_ns
                or local_end_ns > trajectory.end_time_ns
            ):
                raise ValueError("local_window_outside_corrected_run")
            if not _window_has_complete_timestamp_coverage(
                bag_index,
                local_start_ns,
                local_end_ns,
                maximum_gap_ns,
            ):
                raise ValueError("local_window_has_incomplete_telemetry_coverage")

            local_values, local_valid, local_times, channel_names = (
                _resample_all_streams(
                    bag_index,
                    contact_time_ns,
                    local_window_ms,
                    processing_config,
                    trajectory.start_time_ns,
                    trajectory.end_time_ns,
                )
            )
            spectral_values, spectral_valid, spectral_times, _ = (
                _resample_all_streams(
                    bag_index,
                    contact_time_ns,
                    processing_config.spectral_window_ms,
                    processing_config,
                    trajectory.start_time_ns,
                    trajectory.end_time_ns,
                )
            )
            rgb_full = project_full_track_boundaries(
                kinematics_config,
                theta_y_rgb,
                theta_p_rgb,
                rgb_calibration,
                processing_config.track_point_count,
            )
            depth_full = project_full_track_boundaries(
                kinematics_config,
                theta_y_depth,
                theta_p_depth,
                depth_calibration,
                processing_config.track_point_count,
            )
        except ValueError as error:
            reject(pair, str(error))
            continue

        packet_id = _packet_identifier(
            experiment_id,
            run_number,
            theta_y_image,
            pair.rgb_time_ns,
            pair.depth_time_ns,
            dataset_identifier,
        )
        packet_directory = packets_directory / packet_id
        packet_directory.mkdir()
        stored_rgb_encoding = {
            "bgr8": "rgb8",
            "bgra8": "rgba8",
        }.get(rgb_metadata.encoding, rgb_metadata.encoding)
        lap_index = int(math.floor(abs(theta_y_image) / (2.0 * math.pi)))
        metadata = {
            "packet_id": packet_id,
            "experiment_id": experiment_id,
            "run_number": run_number,
            "lap_index": lap_index,
            "rgb_time_ns": pair.rgb_time_ns,
            "depth_time_ns": pair.depth_time_ns,
            "image_time_ns": pair.image_time_ns,
            "sync_error_ns": pair.sync_error_ns,
            "contact_time_ns": contact_time_ns,
            "contact_delay_ns": contact_time_ns - pair.image_time_ns,
            "contact_angular_residual_rad": angular_residual_rad,
            "travel_direction": trajectory.travel_direction,
            "theta_y_image_rad": theta_y_image,
            "theta_y_rgb_rad": theta_y_rgb,
            "theta_y_depth_rad": theta_y_depth,
            "patch_centre_angle_rad": patch_centre_angle_rad,
            "patch_spread_rad": processing_config.alpha_sp_rad,
            "alpha_off_rad": processing_config.alpha_off_rad,
            "local_window_ms": local_window_ms,
            "knee_joint_rgb_rad": knee_joint_rgb,
            "knee_joint_depth_rad": knee_joint_depth,
            "theta_p_rgb_rad": theta_p_rgb,
            "theta_p_depth_rad": theta_p_depth,
            "rgb_calibration": rgb_calibration.to_dict(),
            "depth_calibration": depth_calibration.to_dict(),
            "rgb_frame_id": rgb_metadata.frame_id,
            "depth_frame_id": depth_metadata.frame_id,
            "rgb_source_encoding": rgb_metadata.encoding,
            "rgb_stored_encoding": stored_rgb_encoding,
            "depth_encoding": depth_metadata.encoding,
            "T_rgb_camera_in_base": rgb_projection.T_camera_in_base.tolist(),
            "T_depth_camera_in_base": depth_projection.T_camera_in_base.tolist(),
            "telemetry_channel_names": channel_names,
        }
        _write_json(packet_directory / "metadata.json", metadata)
        np.savez_compressed(
            packet_directory / "geometry.npz",
            rgb_mask=np.packbits(rgb_mask, axis=None),
            rgb_mask_shape=np.asarray(rgb_mask.shape, dtype=np.int64),
            depth_mask=np.packbits(depth_mask, axis=None),
            depth_mask_shape=np.asarray(depth_mask.shape, dtype=np.int64),
            rgb_patch_polygon_pixels=rgb_projection.polygon_pixels,
            depth_patch_polygon_pixels=depth_projection.polygon_pixels,
            patch_inner_points_base_m=rgb_projection.inner_points_base_m,
            patch_outer_points_base_m=rgb_projection.outer_points_base_m,
            rgb_inner_track_pixels=rgb_full[0],
            rgb_inner_track_valid=rgb_full[1],
            rgb_outer_track_pixels=rgb_full[2],
            rgb_outer_track_valid=rgb_full[3],
            depth_inner_track_pixels=depth_full[0],
            depth_inner_track_valid=depth_full[1],
            depth_outer_track_pixels=depth_full[2],
            depth_outer_track_valid=depth_full[3],
        )
        np.savez_compressed(
            packet_directory / "telemetry.npz",
            local_values=local_values,
            local_valid=local_valid,
            local_relative_times_ns=local_times,
            spectral_values=spectral_values,
            spectral_valid=spectral_valid,
            spectral_relative_times_ns=spectral_times,
        )
        image_requests[(RGB_IMAGE_TOPIC, pair.rgb_time_ns)] = _image_writer(
            packet_directory / "rgb.npy", "rgb"
        )
        image_requests[(DEPTH_IMAGE_TOPIC, pair.depth_time_ns)] = _image_writer(
            packet_directory / "depth.npy", "depth"
        )
        manifest_records.append(
            {
                "packet_id": packet_id,
                "image_time_ns": pair.image_time_ns,
                "contact_time_ns": contact_time_ns,
                "theta_y_image_rad": theta_y_image,
                "patch_centre_angle_rad": patch_centre_angle_rad,
                "lap_index": lap_index,
                "sync_error_ns": pair.sync_error_ns,
                "packet_path": str(packet_directory.relative_to(output_directory)),
            }
        )
        last_saved_image_time_ns = pair.image_time_ns

    found_images = extract_selected_images(bag_directory, image_requests)
    missing_images = set(image_requests) - found_images
    if missing_images:
        analysis_report["status"] = "failed_image_recovery"
        analysis_report["missing_selected_image_count"] = len(missing_images)
        _write_json(analysis_report_path, analysis_report)
        raise RuntimeError(
            f"Second bag pass could not recover {len(missing_images)} selected images."
        )

    manifest_path = output_directory / "manifest.jsonl"
    manifest_path.write_text(
        "".join(json.dumps(record) + "\n" for record in manifest_records),
        encoding="utf-8",
    )
    provenance = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "stage": "stage1_complete",
        "dataset_identifier": dataset_identifier,
        "run_directory": str(run_directory),
        "bag_directory": str(bag_directory),
        "experiment_config_source": str(experiment_config_path),
        "experiment_config": experiment_config,
        "run_config": run_config,
        "camera_driver_config": camera_driver_config,
        "processing_config": processing_config.to_dict(),
        "packet_count": len(manifest_records),
    }
    _write_json(output_directory / "provenance.json", provenance)
    quality_report = {
        "rgb_image_count": len(bag_index.rgb_images),
        "depth_image_count": len(bag_index.depth_images),
        "synchronized_pair_count": len(synchronized_pairs),
        "packet_count": len(manifest_records),
        "rejected": analysis_report["rejection_counts"],
    }
    _write_json(output_directory / "quality_report.json", quality_report)
    analysis_report["status"] = "stage1_complete"
    analysis_report["accepted_packet_count"] = len(manifest_records)
    _write_json(analysis_report_path, analysis_report)
    return output_directory


def _read_manifest(dataset_directory: Path) -> list[dict[str, Any]]:
    records = []
    with (dataset_directory / "manifest.jsonl").open(
        "r", encoding="utf-8"
    ) as file:
        for line in file:
            if line.strip():
                records.append(json.loads(line))
    return records


def _unpack_mask(packed: np.ndarray, shape: np.ndarray) -> np.ndarray:
    dimensions = tuple(int(value) for value in shape)
    pixel_count = int(np.prod(dimensions))
    return np.unpackbits(packed, count=pixel_count).reshape(dimensions).astype(bool)


def _mask_crop_origin_uv(mask: np.ndarray) -> list[int]:
    """Return the full-image pixel coordinate of a mask crop's top-left corner."""
    rows, columns = np.nonzero(mask)
    if rows.size == 0:
        raise ValueError("cannot_find_crop_origin_for_empty_mask")
    return [int(columns.min()), int(rows.min())]


def extract_features(
    dataset_directory: Path,
    processing_config: ProcessingConfig | None = None,
) -> Path:
    """Extract patches, point maps, plane slopes, and all-channel spectra."""
    dataset_directory = dataset_directory.resolve()
    provenance_path = dataset_directory / "provenance.json"
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    if processing_config is None:
        processing_config = ProcessingConfig(**provenance["processing_config"])
        processing_config.validate()

    feature_records = []
    for record in _read_manifest(dataset_directory):
        packet_directory = dataset_directory / record["packet_path"]
        stage2_directory = packet_directory / "stage2"
        stage2_directory.mkdir(exist_ok=False)
        metadata = json.loads(
            (packet_directory / "metadata.json").read_text(encoding="utf-8")
        )
        feature_record = {"packet_id": metadata["packet_id"], "valid": False}
        try:
            rgb = np.load(packet_directory / "rgb.npy", allow_pickle=False)
            depth = np.load(packet_directory / "depth.npy", allow_pickle=False)
            geometry = np.load(
                packet_directory / "geometry.npz", allow_pickle=False
            )
            rgb_mask = _unpack_mask(
                geometry["rgb_mask"], geometry["rgb_mask_shape"]
            )
            depth_mask = _unpack_mask(
                geometry["depth_mask"], geometry["depth_mask_shape"]
            )
            rgb_patch, rgb_patch_mask = masked_crop(rgb, rgb_mask)
            depth_patch, depth_patch_mask = masked_crop(depth, depth_mask)
            rgb_crop_origin_uv = _mask_crop_origin_uv(rgb_mask)
            depth_crop_origin_uv = _mask_crop_origin_uv(depth_mask)
            np.save(stage2_directory / "rgb_patch.npy", rgb_patch)
            np.save(stage2_directory / "depth_patch.npy", depth_patch)
            np.save(stage2_directory / "rgb_patch_mask.npy", rgb_patch_mask)
            np.save(stage2_directory / "depth_patch_mask.npy", depth_patch_mask)

            depth_calibration = CameraCalibration.from_dict(
                metadata["depth_calibration"]
            )
            point_map = depth_point_map_in_base(
                depth,
                depth_mask,
                depth_calibration,
                processing_config.depth_scale_m,
                np.asarray(metadata["T_depth_camera_in_base"], dtype=float),
            )
            plane = fit_plane_ransac(
                point_map.points_base_m,
                processing_config.plane_ransac_iterations,
                processing_config.plane_inlier_threshold_m,
                processing_config.minimum_plane_points,
            )
            along_slope, cross_slope = slope_from_plane(
                plane.normal,
                float(metadata["patch_centre_angle_rad"]),
                float(metadata["travel_direction"]),
            )
            np.savez_compressed(
                stage2_directory / "point_map.npz",
                points_base_m=point_map.points_base_m,
                pixel_uv=point_map.pixel_uv,
                depth_m=point_map.depth_m,
                ransac_inlier_mask=plane.inlier_mask,
            )

            telemetry = np.load(
                packet_directory / "telemetry.npz", allow_pickle=False
            )
            (
                frequencies_hz,
                power,
                dominant_frequency_hz,
                dominant_power,
            ) = calculate_spectra(
                telemetry["spectral_values"],
                telemetry["spectral_valid"],
                processing_config.telemetry_resample_hz,
                processing_config.minimum_spectral_valid_fraction,
                processing_config.maximum_interpolation_gap_ms,
            )
            np.savez_compressed(
                stage2_directory / "features.npz",
                plane_normal=plane.normal,
                plane_offset=plane.offset,
                frequencies_hz=frequencies_hz,
                spectral_power=power,
                dominant_frequency_hz=dominant_frequency_hz,
                dominant_power=dominant_power,
            )
            feature_record.update(
                {
                    "valid": True,
                    "along_slope_rad": along_slope,
                    "along_slope_deg": float(np.degrees(along_slope)),
                    "cross_slope_rad": cross_slope,
                    "cross_slope_deg": float(np.degrees(cross_slope)),
                    "cross_slope_positive_direction": "radially_outward",
                    "plane_normal": plane.normal.tolist(),
                    "plane_offset": plane.offset,
                    "plane_rmse_m": plane.rmse_m,
                    "plane_point_count": int(point_map.points_base_m.shape[0]),
                    "plane_inlier_count": int(
                        np.count_nonzero(plane.inlier_mask)
                    ),
                    "plane_inlier_ratio": float(np.mean(plane.inlier_mask)),
                    "valid_depth_fraction": point_map.valid_depth_fraction,
                    "rgb_crop_origin_uv": rgb_crop_origin_uv,
                    "depth_crop_origin_uv": depth_crop_origin_uv,
                    "telemetry_channel_names": metadata[
                        "telemetry_channel_names"
                    ],
                    "dominant_frequency_hz": [
                        [float(value) if np.isfinite(value) else None for value in row]
                        for row in dominant_frequency_hz
                    ],
                    "dominant_power": [
                        [float(value) if np.isfinite(value) else None for value in row]
                        for row in dominant_power
                    ],
                }
            )
        except (ValueError, OSError, KeyError) as error:
            feature_record["error"] = str(error)

        _write_json(stage2_directory / "features.json", feature_record)
        feature_records.append(feature_record)

    features_path = dataset_directory / "features.jsonl"
    features_path.write_text(
        "".join(json.dumps(record) + "\n" for record in feature_records),
        encoding="utf-8",
    )
    provenance["stage"] = "stage2_complete"
    provenance["stage2_completed_utc"] = datetime.now(timezone.utc).isoformat()
    provenance["valid_feature_count"] = sum(
        bool(record["valid"]) for record in feature_records
    )
    _write_json(provenance_path, provenance)
    analysis_report_path = dataset_directory / "packets" / "analysis_report.json"
    if analysis_report_path.is_file():
        analysis_report = json.loads(
            analysis_report_path.read_text(encoding="utf-8")
        )
        analysis_report["status"] = "stage2_complete"
        analysis_report["stage2_valid_packet_count"] = sum(
            bool(record["valid"]) for record in feature_records
        )
        analysis_report["stage2_invalid_packets"] = [
            {
                "packet_id": record["packet_id"],
                "reason": record.get("error", "unknown_stage2_error"),
            }
            for record in feature_records
            if not record["valid"]
        ]
        _write_json(analysis_report_path, analysis_report)
    return dataset_directory
