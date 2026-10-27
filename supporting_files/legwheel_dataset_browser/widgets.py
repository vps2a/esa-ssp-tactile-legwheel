"""Reusable PySide6 and Matplotlib widgets for packet visualization."""

from __future__ import annotations

from typing import Iterable

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHBoxLayout,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QWidget,
)

from .formatting import channel_unit, metadata_rows, short_channel_name
from .model import PacketData, crop_to_mask


PLOT_COLOR = "#2563a6"
SECONDARY_PLOT_COLOR = "#d97706"
PATCH_COLOR = "#ef4444"


class ImageCanvas(FigureCanvasQTAgg):
    """Display a complete image or its masked patch crop."""

    def __init__(self) -> None:
        self.figure = Figure(layout="constrained")
        super().__init__(self.figure)
        self.setMinimumSize(420, 320)

    def show_rgb(
        self,
        image: np.ndarray,
        mask: np.ndarray,
        polygon_pixels: np.ndarray | None,
        *,
        cropped: bool,
    ) -> None:
        """Render the RGB packet image with a clearly marked physical patch."""
        self.figure.clear()
        axis = self.figure.add_subplot(111)
        display = np.asarray(image)
        active_mask = mask
        origin_u = 0
        origin_v = 0
        if cropped:
            display, active_mask, (origin_u, origin_v) = crop_to_mask(display, mask)
            display = np.asarray(display, dtype=float)
            if np.issubdtype(image.dtype, np.integer):
                display /= float(np.iinfo(image.dtype).max)
            display[~active_mask] *= 0.15

        if display.ndim == 2:
            axis.imshow(display, cmap="gray", origin="upper")
        else:
            axis.imshow(display, origin="upper")

        if not cropped:
            overlay = np.zeros((*active_mask.shape, 4), dtype=float)
            overlay[active_mask] = (1.0, 0.1, 0.1, 0.18)
            axis.imshow(overlay, origin="upper")

        self._draw_polygon(axis, polygon_pixels, origin_u, origin_v)
        axis.set_title("RGB patch crop" if cropped else "Complete RGB image and patch")
        axis.set_xlabel("u [pixels]")
        axis.set_ylabel("v [pixels]")
        self.draw_idle()

    def show_depth(
        self,
        depth_m: np.ndarray,
        mask: np.ndarray,
        polygon_pixels: np.ndarray | None,
        limits_m: tuple[float, float],
        *,
        cropped: bool,
    ) -> None:
        """Render metric depth using one explicit colour scale."""
        self.figure.clear()
        axis = self.figure.add_subplot(111)
        display = np.asarray(depth_m, dtype=float)
        active_mask = mask
        origin_u = 0
        origin_v = 0
        if cropped:
            display, active_mask, (origin_u, origin_v) = crop_to_mask(display, mask)

        valid = np.isfinite(display) & (display > 0.0)
        if cropped:
            valid &= active_mask
        masked_depth = np.ma.masked_where(~valid, display)
        image_artist = axis.imshow(
            masked_depth,
            cmap="viridis",
            origin="upper",
            vmin=limits_m[0],
            vmax=limits_m[1],
        )
        if not cropped:
            overlay = np.zeros((*active_mask.shape, 4), dtype=float)
            overlay[active_mask] = (1.0, 0.1, 0.1, 0.18)
            axis.imshow(overlay, origin="upper")

        self._draw_polygon(axis, polygon_pixels, origin_u, origin_v)
        axis.set_title("Depth patch crop" if cropped else "Complete depth map and patch")
        axis.set_xlabel("u [pixels]")
        axis.set_ylabel("v [pixels]")
        self.figure.colorbar(image_artist, ax=axis, label="Depth [m]", shrink=0.86)
        self.draw_idle()

    @staticmethod
    def _draw_polygon(axis, polygon, origin_u: int, origin_v: int) -> None:
        if polygon is None:
            return
        points = np.asarray(polygon, dtype=float)
        if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] != 2:
            return
        points = points - np.asarray([origin_u, origin_v])
        closed = np.vstack((points, points[0]))
        axis.plot(
            closed[:, 0],
            closed[:, 1],
            color=PATCH_COLOR,
            linewidth=2.0,
            label="Physical patch boundary",
        )
        axis.legend(loc="upper right", fontsize=7)


class TelemetryCanvas(FigureCanvasQTAgg):
    """Render a group of telemetry channels as aligned small multiples."""

    def __init__(self, side: str, mode: str) -> None:
        self.side = side
        self.mode = mode
        self.figure = Figure(layout="constrained")
        super().__init__(self.figure)
        self.setMinimumWidth(300)

    def show_packet(self, packet: PacketData) -> None:
        """Draw local, spectral-window, or FFT data for one channel group."""
        self.figure.clear()
        names = packet.channel_names
        indices = self._channel_indices(names)
        if not indices:
            self.figure.text(0.5, 0.5, "No channels in this group", ha="center")
            self.draw_idle()
            return

        if self.mode == "fft":
            frequencies = packet.stage2_arrays.get("frequencies_hz")
            values = packet.stage2_arrays.get("spectral_power")
            valid = None
            horizontal_label = "Frequency [Hz]"
        else:
            prefix = "local" if self.mode == "local" else "spectral"
            values = packet.telemetry.get(f"{prefix}_values")
            valid = packet.telemetry.get(f"{prefix}_valid")
            times_ns = packet.telemetry.get(f"{prefix}_relative_times_ns")
            frequencies = None if times_ns is None else np.asarray(times_ns) / 1e6
            horizontal_label = "Time relative to contact [ms]"

        if values is None or frequencies is None:
            message = "FFT not available (run Stage 2)" if self.mode == "fft" else "Unavailable"
            self.figure.text(0.5, 0.5, message, ha="center", va="center")
            self.draw_idle()
            return

        values = np.asarray(values, dtype=float)
        frequencies = np.asarray(frequencies, dtype=float)
        if values.ndim != 2 or values.shape[1] < max(indices) + 1:
            self.figure.text(0.5, 0.5, "Telemetry dimensions are inconsistent", ha="center")
            self.draw_idle()
            return

        axes = self.figure.subplots(len(indices), 1, sharex=True, squeeze=False)[:, 0]
        for axis, channel_index in zip(axes, indices):
            channel = values[:, channel_index].copy()
            if valid is not None:
                validity = np.asarray(valid, dtype=bool)
                if validity.shape == values.shape:
                    channel[~validity[:, channel_index]] = np.nan
            axis.plot(frequencies, channel, color=PLOT_COLOR, linewidth=0.9)
            if self.mode != "fft":
                axis.axvline(0.0, color=SECONDARY_PLOT_COLOR, linewidth=0.7, alpha=0.8)
            unit = channel_unit(names[channel_index])
            axis.set_ylabel(unit, fontsize=7)
            axis.set_title(short_channel_name(names[channel_index]), fontsize=8, loc="left")
            axis.grid(True, alpha=0.22, linewidth=0.5)
            axis.tick_params(labelsize=7)
        axes[-1].set_xlabel(horizontal_label, fontsize=8)
        self.draw_idle()

    def _channel_indices(self, names: Iterable[str]) -> list[int]:
        names = list(names)
        if self.side == "imu":
            return [index for index, name in enumerate(names) if name.startswith("imu.")]
        return [index for index, name in enumerate(names) if not name.startswith("imu.")]


class TelemetryPanel(QTabWidget):
    """Tabbed telemetry sidebar for either motor/joint or IMU channels."""

    MODES = (("Local", "local"), ("Spectral window", "spectral"), ("FFT", "fft"))

    def __init__(self, side: str) -> None:
        super().__init__()
        self.canvases: list[TelemetryCanvas] = []
        for label, mode in self.MODES:
            canvas = TelemetryCanvas(side, mode)
            self.canvases.append(canvas)
            self.addTab(canvas, label)
        self.setMinimumWidth(300)

    def show_packet(self, packet: PacketData) -> None:
        for canvas in self.canvases:
            canvas.show_packet(packet)


class SlopeSummary(QWidget):
    """Keep the two ML slope targets prominent above all central views."""

    def __init__(self) -> None:
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        self.along_label = self._important_label("Along-travel slope\nUnavailable")
        self.cross_label = self._important_label("Cross-track slope\nUnavailable")
        self.diagnostic_label = QLabel("Stage 2 diagnostics unavailable")
        self.diagnostic_label.setWordWrap(True)
        self.diagnostic_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.along_label, 1)
        layout.addWidget(self.cross_label, 1)
        layout.addWidget(self.diagnostic_label, 2)

    @staticmethod
    def _important_label(text: str) -> QLabel:
        label = QLabel(text)
        font = QFont(label.font())
        font.setBold(True)
        font.setPointSize(font.pointSize() + 4)
        label.setFont(font)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        return label

    def show_packet(self, packet: PacketData) -> None:
        features = packet.features or {}
        if packet.stage2_available:
            self.along_label.setText(
                f"Along-travel slope\n{float(features['along_slope_deg']):+.3f}°"
            )
            self.cross_label.setText(
                f"Cross-track slope\n{float(features['cross_slope_deg']):+.3f}°"
            )
            self.diagnostic_label.setText(
                "Plane RMSE: "
                f"{float(features.get('plane_rmse_m', float('nan'))):.5f} m  ·  "
                f"Inliers: {features.get('plane_inlier_count', '?')}/"
                f"{features.get('plane_point_count', '?')}  ·  "
                "Valid depth: "
                f"{100.0 * float(features.get('valid_depth_fraction', 0.0)):.1f}%"
            )
        else:
            self.along_label.setText("Along-travel slope\nUnavailable")
            self.cross_label.setText("Cross-track slope\nUnavailable")
            self.diagnostic_label.setText(
                str(features.get("error", "Stage 2 has not been run for this packet"))
            )


class PointMapCanvas(FigureCanvasQTAgg):
    """Show the Stage 2 point map, RANSAC inliers, patch, and plane normal."""

    MAX_DISPLAY_POINTS = 6000

    def __init__(self) -> None:
        self.figure = Figure(layout="constrained")
        super().__init__(self.figure)

    def show_packet(self, packet: PacketData) -> None:
        self.figure.clear()
        points = packet.point_map.get("points_base_m")
        inliers = packet.point_map.get("ransac_inlier_mask")
        if not packet.stage2_available or points is None or inliers is None:
            self.figure.text(
                0.5,
                0.5,
                "3D diagnostic unavailable\nRun Stage 2 for this packet.",
                ha="center",
                va="center",
            )
            self.draw_idle()
            return

        axis = self.figure.add_subplot(111, projection="3d")
        points = np.asarray(points, dtype=float)
        inliers = np.asarray(inliers, dtype=bool)
        if points.shape[0] > self.MAX_DISPLAY_POINTS:
            selection = np.linspace(
                0,
                points.shape[0] - 1,
                self.MAX_DISPLAY_POINTS,
                dtype=int,
            )
            points = points[selection]
            inliers = inliers[selection]

        outlier_points = points[~inliers]
        inlier_points = points[inliers]
        if outlier_points.size:
            axis.scatter(
                outlier_points[:, 0],
                outlier_points[:, 1],
                outlier_points[:, 2],
                s=2,
                alpha=0.25,
                color=PATCH_COLOR,
                label="Rejected points",
            )
        if inlier_points.size:
            axis.scatter(
                inlier_points[:, 0],
                inlier_points[:, 1],
                inlier_points[:, 2],
                s=3,
                alpha=0.65,
                color=PLOT_COLOR,
                label="Plane inliers",
            )

        for key, label in (
            ("patch_inner_points_base_m", "Inner patch boundary"),
            ("patch_outer_points_base_m", "Outer patch boundary"),
        ):
            boundary = packet.geometry.get(key)
            if boundary is not None:
                boundary = np.asarray(boundary)
                axis.plot(
                    boundary[:, 0],
                    boundary[:, 1],
                    boundary[:, 2],
                    linewidth=2.0,
                    label=label,
                )

        features = packet.features or {}
        if inlier_points.size and "plane_normal" in features:
            centre = np.mean(inlier_points, axis=0)
            normal = np.asarray(features["plane_normal"], dtype=float)
            axis.quiver(*centre, *normal, length=0.12, color=SECONDARY_PLOT_COLOR)

        axis.set_xlabel("Base X [m]")
        axis.set_ylabel("Base Y [m]")
        axis.set_zlabel("Base Z [m]")
        axis.set_title("Depth point map and fitted-plane diagnostics")
        axis.legend(loc="upper right", fontsize=8)
        self._set_equal_scale(axis, points)
        self.draw_idle()

    @staticmethod
    def _set_equal_scale(axis, points: np.ndarray) -> None:
        if not points.size:
            return
        lower = np.nanmin(points, axis=0)
        upper = np.nanmax(points, axis=0)
        centre = (lower + upper) / 2.0
        radius = max(float(np.max(upper - lower)) / 2.0, 0.01)
        axis.set_xlim(centre[0] - radius, centre[0] + radius)
        axis.set_ylim(centre[1] - radius, centre[1] + radius)
        axis.set_zlim(centre[2] - radius, centre[2] + radius)


class MetadataTable(QTableWidget):
    """Read-only table of the most useful packet and plane-fit metadata."""

    def __init__(self) -> None:
        super().__init__(0, 2)
        self.setHorizontalHeaderLabels(("Field", "Value"))
        self.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.verticalHeader().setVisible(False)
        self.horizontalHeader().setStretchLastSection(True)
        self.setAlternatingRowColors(True)

    def show_packet(self, packet: PacketData) -> None:
        rows = metadata_rows(packet)
        self.setRowCount(len(rows))
        for row, (name, value) in enumerate(rows):
            self.setItem(row, 0, QTableWidgetItem(name))
            self.setItem(row, 1, QTableWidgetItem(value))
        self.resizeColumnToContents(0)
