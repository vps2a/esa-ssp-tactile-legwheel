"""Stage 1 packet isolation and Stage 2 feature extraction."""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
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
    depth_points_in_base,
    masked_crop,
    polygon_mask,
)
from legwheel_dataset.synchronization import (
    find_future_crossing_time,
    interpolate_scalar,
    pair_image_timestamps,
    resample_channels,
)
from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.projection import CameraCalibration
from legwheel_kinematics.track import project_track
from legwheel_kinematics.transforms import forward_kinematics, theta_p_from_config


def _load_yaml(path: Path) -> dict[str, Any]:
    import yaml

    with path.open("r", encoding="utf-8") as file:
        values = yaml.safe_load(file)
    if not isinstance(values, dict):
        raise ValueError(f"Configuration must contain a mapping: {path}")
    return values


def find_experiment_snapshot(run_directory: Path) -> Path:
    """Find the unique experiment snapshot saved by RunRecorder."""
    candidates = sorted(
        run_directory.glob("experiment_config_run_*_snapshot.yaml")
    )
    if len(candidates) != 1:
        raise ValueError(
            "Run directory must contain exactly one experiment configuration "
            f"snapshot; found {len(candidates)}."
        )
    return candidates[0]


def _dataset_identifier(
    experiment_config: dict[str, Any],
    processing_config: ProcessingConfig,
) -> str:
    payload = json.dumps(
        {
            "experiment": experiment_config,
            "processing": processing_config.to_dict(),
        },
        sort_keys=True,
        separators=(",", ":"),
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
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    sampled_streams = []
    sampled_masks = []
    channel_names = []
    relative_times_ns = None
    for stream_name in ("leg", "wheel", "rotation", "imu"):
        stream = bag_index.streams[stream_name]
        source_times_ns, source_values = stream.arrays()
        if source_times_ns.size < 2:
            raise ValueError(f"Telemetry stream '{stream_name}' has too few samples.")
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
    return (
        np.column_stack(sampled_streams),
        np.column_stack(sampled_masks),
        relative_times_ns,
        channel_names,
    )


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


def isolate_packets(
    run_directory: Path,
    processing_config: ProcessingConfig,
    experiment_config_path: Path | None = None,
    output_directory: Path | None = None,
) -> Path:
    """Convert synchronized bag data into self-contained Stage 1 packets."""
    run_directory = run_directory.resolve()
    if experiment_config_path is None:
        experiment_config_path = find_experiment_snapshot(run_directory)
    experiment_config_path = experiment_config_path.resolve()
    experiment_config = _load_yaml(experiment_config_path)
    kinematics_config = KinematicsConfig.from_experiment_dict(experiment_config)
    bag_directory = run_directory / "rosbag"
    if not bag_directory.is_dir():
        raise FileNotFoundError(f"Rosbag directory does not exist: {bag_directory}")

    dataset_identifier = _dataset_identifier(
        experiment_config,
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
    rgb_times_ns = np.asarray(
        [image.timestamp_ns for image in bag_index.rgb_images],
        dtype=np.int64,
    )
    depth_times_ns = np.asarray(
        [image.timestamp_ns for image in bag_index.depth_images],
        dtype=np.int64,
    )
    pairs = pair_image_timestamps(
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
    rotation_times_ns, rotation_values = bag_index.streams["rotation"].arrays()
    rotation_column = bag_index.streams["rotation"].channel_names.index(
        "rotation_joint.position"
    )
    rotation_times_ns, rotation_angles = _finite_column(
        rotation_times_ns,
        rotation_values,
        rotation_column,
    )
    if knee_times_ns.size < 2:
        raise ValueError("Bag contains too few valid knee_joint samples.")
    if rotation_times_ns.size < 2:
        raise ValueError("Bag contains too few valid rotation_joint samples.")
    rotation_angles = np.unwrap(rotation_angles)

    manifest_records = []
    rejected: dict[str, int] = {}
    image_requests = {}

    def reject(reason: str) -> None:
        rejected[reason] = rejected.get(reason, 0) + 1

    for pair in pairs:
        try:
            knee_joint = interpolate_scalar(
                knee_times_ns,
                knee_values,
                pair.image_time_ns,
            )
            theta_p = theta_p_from_config(knee_joint, kinematics_config)
            rgb_calibration = bag_index.rgb_calibrations.at(pair.rgb_time_ns)
            depth_calibration = bag_index.depth_calibrations.at(
                pair.depth_time_ns
            )
            rgb_projection = project_track(
                kinematics_config,
                theta_p,
                rgb_calibration,
                processing_config.track_point_count,
            )
            depth_projection = project_track(
                kinematics_config,
                theta_p,
                depth_calibration,
                processing_config.track_point_count,
            )
            if (
                rgb_projection.polygon_pixels.size == 0
                or depth_projection.polygon_pixels.size == 0
            ):
                raise ValueError("track_not_visible")
            contact_time_ns, travel_direction = find_future_crossing_time(
                rotation_times_ns,
                rotation_angles,
                pair.image_time_ns,
                rgb_projection.midpoint_angle_rad,
            )
            local_values, local_valid, local_times, channel_names = (
                _resample_all_streams(
                    bag_index,
                    contact_time_ns,
                    processing_config.local_window_ms,
                    processing_config,
                )
            )
            spectral_values, spectral_valid, spectral_times, _ = (
                _resample_all_streams(
                    bag_index,
                    contact_time_ns,
                    processing_config.spectral_window_ms,
                    processing_config,
                )
            )
        except ValueError as error:
            reject(str(error))
            continue

        packet_id = f"packet_{len(manifest_records):06d}"
        packet_directory = packets_directory / packet_id
        packet_directory.mkdir()
        T_cam = forward_kinematics(kinematics_config, theta_p)
        rgb_metadata = bag_index.rgb_images[pair.rgb_index]
        depth_metadata = bag_index.depth_images[pair.depth_index]
        stored_rgb_encoding = {
            "bgr8": "rgb8",
            "bgra8": "rgba8",
        }.get(rgb_metadata.encoding, rgb_metadata.encoding)
        metadata = {
            "packet_id": packet_id,
            "rgb_time_ns": pair.rgb_time_ns,
            "depth_time_ns": pair.depth_time_ns,
            "image_time_ns": pair.image_time_ns,
            "sync_error_ns": pair.sync_error_ns,
            "contact_time_ns": contact_time_ns,
            "contact_delay_ns": contact_time_ns - pair.image_time_ns,
            "travel_direction": travel_direction,
            "knee_joint": knee_joint,
            "theta_p": theta_p,
            "track_midpoint_angle_rad": rgb_projection.midpoint_angle_rad,
            "rgb_polygon_pixels": rgb_projection.polygon_pixels.tolist(),
            "depth_polygon_pixels": depth_projection.polygon_pixels.tolist(),
            "rgb_calibration": rgb_calibration.to_dict(),
            "depth_calibration": depth_calibration.to_dict(),
            "rgb_source_encoding": rgb_metadata.encoding,
            "rgb_stored_encoding": stored_rgb_encoding,
            "depth_encoding": depth_metadata.encoding,
            "T_cam": T_cam.tolist(),
            "telemetry_channel_names": channel_names,
        }
        (packet_directory / "metadata.json").write_text(
            json.dumps(metadata, indent=2),
            encoding="utf-8",
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
            packet_directory / "rgb.npy",
            "rgb",
        )
        image_requests[(DEPTH_IMAGE_TOPIC, pair.depth_time_ns)] = _image_writer(
            packet_directory / "depth.npy",
            "depth",
        )
        manifest_records.append(
            {
                "packet_id": packet_id,
                "image_time_ns": pair.image_time_ns,
                "contact_time_ns": contact_time_ns,
                "sync_error_ns": pair.sync_error_ns,
                "packet_path": str(packet_directory.relative_to(output_directory)),
            }
        )

    found_images = extract_selected_images(bag_directory, image_requests)
    missing_images = set(image_requests) - found_images
    if missing_images:
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
        "processing_config": processing_config.to_dict(),
        "packet_count": len(manifest_records),
    }
    (output_directory / "provenance.json").write_text(
        json.dumps(provenance, indent=2),
        encoding="utf-8",
    )
    quality_report = {
        "rgb_image_count": len(bag_index.rgb_images),
        "depth_image_count": len(bag_index.depth_images),
        "synchronized_pair_count": len(pairs),
        "packet_count": len(manifest_records),
        "rejected": rejected,
    }
    (output_directory / "quality_report.json").write_text(
        json.dumps(quality_report, indent=2),
        encoding="utf-8",
    )
    return output_directory


def _read_manifest(dataset_directory: Path) -> list[dict[str, Any]]:
    records = []
    with (dataset_directory / "manifest.jsonl").open(
        "r",
        encoding="utf-8",
    ) as file:
        for line in file:
            if line.strip():
                records.append(json.loads(line))
    return records


def extract_features(
    dataset_directory: Path,
    processing_config: ProcessingConfig | None = None,
) -> Path:
    """Extract track patches, plane slopes, and spectra from Stage 1 packets."""
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
            rgb_polygon = np.asarray(metadata["rgb_polygon_pixels"], dtype=float)
            depth_polygon = np.asarray(
                metadata["depth_polygon_pixels"],
                dtype=float,
            )
            rgb_mask = polygon_mask(rgb.shape[0], rgb.shape[1], rgb_polygon)
            depth_mask = polygon_mask(
                depth.shape[0],
                depth.shape[1],
                depth_polygon,
            )
            rgb_patch, rgb_patch_mask = masked_crop(rgb, rgb_mask)
            depth_patch, depth_patch_mask = masked_crop(depth, depth_mask)
            np.save(stage2_directory / "rgb_patch.npy", rgb_patch)
            np.save(stage2_directory / "depth_patch.npy", depth_patch)
            np.save(stage2_directory / "rgb_patch_mask.npy", rgb_patch_mask)
            np.save(stage2_directory / "depth_patch_mask.npy", depth_patch_mask)

            depth_calibration = CameraCalibration.from_dict(
                metadata["depth_calibration"]
            )
            points_base, valid_depth_fraction = depth_points_in_base(
                depth,
                depth_mask,
                depth_calibration,
                processing_config.depth_scale_m,
                np.asarray(metadata["T_cam"], dtype=float),
            )
            plane = fit_plane_ransac(
                points_base,
                processing_config.plane_ransac_iterations,
                processing_config.plane_inlier_threshold_m,
                processing_config.minimum_plane_points,
            )
            along_slope, cross_slope = slope_from_plane(
                plane.normal,
                float(metadata["track_midpoint_angle_rad"]),
                float(metadata["travel_direction"]),
            )

            telemetry = np.load(packet_directory / "telemetry.npz")
            frequencies_hz, power, dominant_frequency_hz = calculate_spectra(
                telemetry["spectral_values"],
                telemetry["spectral_valid"],
                processing_config.telemetry_resample_hz,
                processing_config.minimum_spectral_valid_fraction,
            )
            np.savez_compressed(
                stage2_directory / "features.npz",
                plane_normal=plane.normal,
                plane_offset=plane.offset,
                frequencies_hz=frequencies_hz,
                spectral_power=power,
                dominant_frequency_hz=dominant_frequency_hz,
            )
            feature_record.update({
                "valid": True,
                "along_slope_rad": along_slope,
                "along_slope_deg": float(np.degrees(along_slope)),
                "cross_slope_rad": cross_slope,
                "cross_slope_deg": float(np.degrees(cross_slope)),
                "plane_normal": plane.normal.tolist(),
                "plane_offset": plane.offset,
                "plane_rmse_m": plane.rmse_m,
                "plane_point_count": int(points_base.shape[0]),
                "plane_inlier_count": int(np.count_nonzero(plane.inlier_mask)),
                "plane_inlier_ratio": float(np.mean(plane.inlier_mask)),
                "valid_depth_fraction": valid_depth_fraction,
                "dominant_frequency_hz": [
                    float(value) if np.isfinite(value) else None
                    for value in dominant_frequency_hz
                ],
            })
        except (ValueError, OSError) as error:
            feature_record["error"] = str(error)

        (stage2_directory / "features.json").write_text(
            json.dumps(feature_record, indent=2),
            encoding="utf-8",
        )
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
    provenance_path.write_text(
        json.dumps(provenance, indent=2),
        encoding="utf-8",
    )
    return dataset_directory
