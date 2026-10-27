"""Read and validate derived LegWheel packet datasets without modifying them."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray


class DatasetFormatError(ValueError):
    """Raised when a selected directory is not a usable derived dataset."""


@dataclass(frozen=True)
class PacketData:
    """All files required to display one packet.

    Arrays are loaded into memory and all source files are closed before this
    object is returned. The browser never opens dataset files for writing.
    """

    index: int
    packet_directory: Path
    manifest: dict[str, Any]
    metadata: dict[str, Any]
    features: dict[str, Any] | None
    rgb: NDArray
    depth: NDArray
    rgb_mask: NDArray[np.bool_]
    depth_mask: NDArray[np.bool_]
    geometry: dict[str, NDArray]
    telemetry: dict[str, NDArray]
    stage2_arrays: dict[str, NDArray]
    point_map: dict[str, NDArray]

    @property
    def stage2_available(self) -> bool:
        """Return whether Stage 2 produced valid features for this packet."""
        return bool(self.features and self.features.get("valid"))

    @property
    def channel_names(self) -> list[str]:
        """Return the telemetry column names in their stored order."""
        return [str(name) for name in self.metadata.get("telemetry_channel_names", [])]


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise DatasetFormatError(f"Required file is missing: {path}") from error
    except json.JSONDecodeError as error:
        raise DatasetFormatError(f"Invalid JSON in {path}: {error}") from error
    if not isinstance(value, dict):
        raise DatasetFormatError(f"Expected a JSON object in {path}.")
    return value


def _read_npz(path: Path, *, required: bool = True) -> dict[str, NDArray]:
    if not path.is_file():
        if required:
            raise DatasetFormatError(f"Required file is missing: {path}")
        return {}
    try:
        with np.load(path, allow_pickle=False) as archive:
            return {name: np.asarray(archive[name]) for name in archive.files}
    except (OSError, ValueError) as error:
        raise DatasetFormatError(f"Cannot read NumPy archive {path}: {error}") from error


def unpack_mask(packed: NDArray, shape: NDArray) -> NDArray[np.bool_]:
    """Restore a bit-packed Stage 1 image mask."""
    dimensions = tuple(int(value) for value in np.asarray(shape).reshape(-1))
    if len(dimensions) != 2 or min(dimensions) <= 0:
        raise DatasetFormatError(f"Invalid mask dimensions: {dimensions}")
    pixel_count = int(np.prod(dimensions))
    bits = np.unpackbits(np.asarray(packed, dtype=np.uint8), count=pixel_count)
    return bits.reshape(dimensions).astype(bool)


def crop_to_mask(
    image: NDArray,
    mask: NDArray[np.bool_],
) -> tuple[NDArray, NDArray[np.bool_], tuple[int, int]]:
    """Crop an image to its mask and return the full-image (u, v) origin."""
    rows, columns = np.nonzero(mask)
    if rows.size == 0:
        raise DatasetFormatError("The packet contains an empty patch mask.")
    row_slice = slice(int(rows.min()), int(rows.max()) + 1)
    column_slice = slice(int(columns.min()), int(columns.max()) + 1)
    origin_uv = (int(columns.min()), int(rows.min()))
    return image[row_slice, column_slice], mask[row_slice, column_slice], origin_uv


class LegWheelDataset:
    """Chronological, read-only view of one configuration-hash directory."""

    def __init__(
        self,
        root: Path,
        provenance: dict[str, Any],
        records: list[dict[str, Any]],
    ) -> None:
        self.root = root
        self.provenance = provenance
        self.records = records
        self._depth_limits_m: tuple[float, float] | None = None

        first_metadata = _read_json(self._packet_directory(records[0]) / "metadata.json")
        self.experiment_id = str(first_metadata.get("experiment_id", "unknown"))
        self.run_number = first_metadata.get("run_number", "unknown")
        processing = provenance.get("processing_config", {})
        self.depth_scale_m = float(processing.get("depth_scale_m", 0.001))
        self.telemetry_resample_hz = float(
            processing.get("telemetry_resample_hz", 100.0)
        )

    @classmethod
    def open(cls, directory: Path | str) -> "LegWheelDataset":
        """Open and validate a configuration-hash dataset directory."""
        root = Path(directory).expanduser().resolve()
        if not root.is_dir():
            raise DatasetFormatError(f"Dataset directory does not exist: {root}")

        provenance = _read_json(root / "provenance.json")
        manifest_path = root / "manifest.jsonl"
        try:
            lines = manifest_path.read_text(encoding="utf-8").splitlines()
        except FileNotFoundError as error:
            raise DatasetFormatError(f"Required file is missing: {manifest_path}") from error

        records: list[dict[str, Any]] = []
        for line_number, line in enumerate(lines, start=1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise DatasetFormatError(
                    f"Invalid manifest line {line_number}: {error}"
                ) from error
            if not isinstance(record, dict) or "packet_path" not in record:
                raise DatasetFormatError(
                    f"Manifest line {line_number} has no packet_path."
                )
            records.append(record)

        if not records:
            raise DatasetFormatError("The selected dataset manifest contains no packets.")
        records.sort(key=lambda record: int(record.get("image_time_ns", 0)))
        return cls(root, provenance, records)

    def __len__(self) -> int:
        return len(self.records)

    def _packet_directory(self, record: dict[str, Any]) -> Path:
        packet_directory = (self.root / str(record["packet_path"])).resolve()
        try:
            packet_directory.relative_to(self.root)
        except ValueError as error:
            raise DatasetFormatError(
                f"Packet path escapes the dataset directory: {record['packet_path']}"
            ) from error
        if not packet_directory.is_dir():
            raise DatasetFormatError(f"Packet directory is missing: {packet_directory}")
        return packet_directory

    def load_packet(self, index: int) -> PacketData:
        """Load one packet by chronological index."""
        if not 0 <= index < len(self):
            raise IndexError(f"Packet index {index} is outside 0..{len(self) - 1}.")
        record = self.records[index]
        packet_directory = self._packet_directory(record)

        metadata = _read_json(packet_directory / "metadata.json")
        try:
            rgb = np.load(packet_directory / "rgb.npy", allow_pickle=False)
            depth = np.load(packet_directory / "depth.npy", allow_pickle=False)
        except (FileNotFoundError, OSError, ValueError) as error:
            raise DatasetFormatError(
                f"Cannot load packet images from {packet_directory}: {error}"
            ) from error

        geometry = _read_npz(packet_directory / "geometry.npz")
        telemetry = _read_npz(packet_directory / "telemetry.npz")
        required_geometry = {
            "rgb_mask",
            "rgb_mask_shape",
            "depth_mask",
            "depth_mask_shape",
        }
        missing_geometry = required_geometry - geometry.keys()
        if missing_geometry:
            raise DatasetFormatError(
                "geometry.npz is missing: " + ", ".join(sorted(missing_geometry))
            )

        rgb_mask = unpack_mask(geometry["rgb_mask"], geometry["rgb_mask_shape"])
        depth_mask = unpack_mask(geometry["depth_mask"], geometry["depth_mask_shape"])
        if rgb_mask.shape != rgb.shape[:2] or depth_mask.shape != depth.shape[:2]:
            raise DatasetFormatError("Stored image and patch-mask dimensions do not match.")

        stage2_directory = packet_directory / "stage2"
        features_path = stage2_directory / "features.json"
        features = _read_json(features_path) if features_path.is_file() else None
        stage2_arrays = _read_npz(stage2_directory / "features.npz", required=False)
        point_map = _read_npz(stage2_directory / "point_map.npz", required=False)

        return PacketData(
            index=index,
            packet_directory=packet_directory,
            manifest=record,
            metadata=metadata,
            features=features,
            rgb=np.asarray(rgb),
            depth=np.asarray(depth),
            rgb_mask=rgb_mask,
            depth_mask=depth_mask,
            geometry=geometry,
            telemetry=telemetry,
            stage2_arrays=stage2_arrays,
            point_map=point_map,
        )

    def depth_to_metres(self, depth: NDArray) -> NDArray[np.float64]:
        """Apply the processor's depth scaling convention."""
        values = np.asarray(depth, dtype=float)
        if np.issubdtype(np.asarray(depth).dtype, np.integer):
            values *= self.depth_scale_m
        return values

    def global_depth_limits_m(self) -> tuple[float, float]:
        """Return fixed valid-depth limits across all complete packet images."""
        if self._depth_limits_m is not None:
            return self._depth_limits_m

        minimum = np.inf
        maximum = -np.inf
        for record in self.records:
            depth_path = self._packet_directory(record) / "depth.npy"
            try:
                depth = np.load(depth_path, mmap_mode="r", allow_pickle=False)
            except (OSError, ValueError) as error:
                raise DatasetFormatError(f"Cannot scan {depth_path}: {error}") from error
            depth_m = self.depth_to_metres(depth)
            valid = depth_m[np.isfinite(depth_m) & (depth_m > 0.0)]
            if valid.size:
                minimum = min(minimum, float(np.min(valid)))
                maximum = max(maximum, float(np.max(valid)))

        if not np.isfinite(minimum) or not np.isfinite(maximum):
            self._depth_limits_m = (0.0, 1.0)
        elif minimum == maximum:
            padding = max(0.01, abs(minimum) * 0.01)
            self._depth_limits_m = (minimum - padding, maximum + padding)
        else:
            self._depth_limits_m = (minimum, maximum)
        return self._depth_limits_m
