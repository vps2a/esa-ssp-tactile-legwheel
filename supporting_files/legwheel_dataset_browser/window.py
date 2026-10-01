"""Main window for the read-only LegWheel packet browser."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PySide6.QtCore import QSettings, Qt, QTimer
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .model import DatasetFormatError, LegWheelDataset, PacketData
from .widgets import (
    ImageCanvas,
    MetadataTable,
    PointMapCanvas,
    SlopeSummary,
    TelemetryPanel,
)


class DatasetBrowserWindow(QMainWindow):
    """Resizable, keyboard-navigable, read-only packet inspection window."""

    SETTINGS_LAST_DATASET = "last_dataset_directory"

    def __init__(self, initial_dataset: Path | None = None) -> None:
        super().__init__()
        self.setWindowTitle("LegWheel Dataset Browser")
        self.resize(1920, 1080)
        self.setMinimumSize(1280, 720)

        self.settings = QSettings()
        self.dataset: LegWheelDataset | None = None
        self.packet: PacketData | None = None
        self.packet_index = 0
        self.fixed_depth_limits_m = (0.0, 1.0)

        self._build_interface()
        self._connect_navigation()

        candidate = initial_dataset
        if candidate is None:
            saved = self.settings.value(self.SETTINGS_LAST_DATASET, "", type=str)
            if saved:
                candidate = Path(saved)
        if candidate is not None and candidate.is_dir():
            if not self.open_dataset(candidate, show_error=False):
                QTimer.singleShot(0, self.choose_dataset)
        else:
            QTimer.singleShot(0, self.choose_dataset)

    def _build_interface(self) -> None:
        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)

        header = QHBoxLayout()
        self.identity_label = QLabel("No dataset selected")
        identity_font = self.identity_label.font()
        identity_font.setBold(True)
        identity_font.setPointSize(identity_font.pointSize() + 3)
        self.identity_label.setFont(identity_font)
        self.path_label = QLabel("")
        self.path_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.change_dataset_button = QPushButton("Change dataset…")
        self.change_dataset_button.clicked.connect(self.choose_dataset)
        header.addWidget(self.identity_label)
        header.addWidget(self.path_label, 1)
        header.addWidget(self.change_dataset_button)
        outer.addLayout(header)

        self.slope_summary = SlopeSummary()
        outer.addWidget(self.slope_summary)

        body = QSplitter(Qt.Orientation.Horizontal)
        self.left_telemetry = TelemetryPanel("motor")
        self.right_telemetry = TelemetryPanel("imu")
        self.left_telemetry.currentChanged.connect(self.right_telemetry.setCurrentIndex)
        self.right_telemetry.currentChanged.connect(self.left_telemetry.setCurrentIndex)

        self.central_tabs = QTabWidget()
        self.central_tabs.addTab(self._build_images_tab(), "Images and patches")
        self.point_map_canvas = PointMapCanvas()
        self.central_tabs.addTab(self.point_map_canvas, "3D diagnostic")
        self.metadata_table = MetadataTable()
        self.central_tabs.addTab(self.metadata_table, "Metadata")

        body.addWidget(self.left_telemetry)
        body.addWidget(self.central_tabs)
        body.addWidget(self.right_telemetry)
        body.setStretchFactor(0, 2)
        body.setStretchFactor(1, 5)
        body.setStretchFactor(2, 2)
        body.setSizes((350, 1050, 350))
        outer.addWidget(body, 1)

        navigation = QHBoxLayout()
        self.previous_button = QPushButton("← Previous packet")
        self.next_button = QPushButton("Next packet →")
        self.packet_position_label = QLabel("Packet 0 of 0")
        self.packet_position_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        navigation.addWidget(self.previous_button)
        navigation.addStretch(1)
        navigation.addWidget(self.packet_position_label)
        navigation.addStretch(1)
        navigation.addWidget(self.next_button)
        outer.addLayout(navigation)

        self.setCentralWidget(central)
        self.statusBar().showMessage("Select a configuration-hash dataset directory.")

    def _build_images_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        controls = QHBoxLayout()
        self.crop_checkbox = QCheckBox("Show cropped masked patches")
        self.auto_depth_checkbox = QCheckBox("Auto-scale depth for current packet")
        controls.addWidget(self.crop_checkbox)
        controls.addWidget(self.auto_depth_checkbox)
        controls.addStretch(1)
        layout.addLayout(controls)

        images = QSplitter(Qt.Orientation.Horizontal)
        self.rgb_canvas = ImageCanvas()
        self.depth_canvas = ImageCanvas()
        images.addWidget(self.rgb_canvas)
        images.addWidget(self.depth_canvas)
        images.setSizes((700, 700))
        layout.addWidget(images, 1)
        return tab

    def _connect_navigation(self) -> None:
        self.previous_button.clicked.connect(self.previous_packet)
        self.next_button.clicked.connect(self.next_packet)
        self.crop_checkbox.toggled.connect(self._refresh_images)
        self.auto_depth_checkbox.toggled.connect(self._refresh_images)
        QShortcut(QKeySequence(Qt.Key.Key_Left), self).activated.connect(
            self.previous_packet
        )
        QShortcut(QKeySequence(Qt.Key.Key_Right), self).activated.connect(
            self.next_packet
        )

    def choose_dataset(self) -> None:
        """Ask for a configuration-hash directory and retain the last choice."""
        start = self.settings.value(self.SETTINGS_LAST_DATASET, "", type=str)
        if not start or not Path(start).is_dir():
            start = str(Path.home())
        selected = QFileDialog.getExistingDirectory(
            self,
            "Select derived dataset configuration-hash directory",
            start,
            QFileDialog.Option.ShowDirsOnly,
        )
        if selected:
            self.open_dataset(Path(selected), show_error=True)

    def open_dataset(self, directory: Path, *, show_error: bool = True) -> bool:
        """Validate and display a dataset without changing any source files."""
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            dataset = LegWheelDataset.open(directory)
            depth_limits = dataset.global_depth_limits_m()
            first_packet = dataset.load_packet(0)
        except (DatasetFormatError, OSError, ValueError) as error:
            if show_error:
                QMessageBox.critical(self, "Cannot open dataset", str(error))
            self.statusBar().showMessage(f"Could not open dataset: {error}")
            return False
        finally:
            QApplication.restoreOverrideCursor()

        self.dataset = dataset
        self.fixed_depth_limits_m = depth_limits
        self.packet_index = 0
        self.settings.setValue(self.SETTINGS_LAST_DATASET, str(dataset.root))
        self.identity_label.setText(
            f"Experiment {dataset.experiment_id}  ·  Run {dataset.run_number}"
        )
        self.path_label.setText(f"Dataset: {dataset.root.name}")
        self.path_label.setToolTip(str(dataset.root))
        self._show_loaded_packet(first_packet)
        self.statusBar().showMessage(
            f"Loaded {len(dataset)} packets from {dataset.root}", 8000
        )
        return True

    def show_packet(self, index: int) -> None:
        if self.dataset is None or not 0 <= index < len(self.dataset):
            return
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            packet = self.dataset.load_packet(index)
        except (DatasetFormatError, OSError, ValueError) as error:
            QMessageBox.critical(self, "Cannot load packet", str(error))
            self.statusBar().showMessage(f"Packet load failed: {error}")
            return
        finally:
            QApplication.restoreOverrideCursor()
        self.packet_index = index
        self._show_loaded_packet(packet)

    def _show_loaded_packet(self, packet: PacketData) -> None:
        self.packet = packet
        self.slope_summary.show_packet(packet)
        self.left_telemetry.show_packet(packet)
        self.right_telemetry.show_packet(packet)
        self.point_map_canvas.show_packet(packet)
        self.metadata_table.show_packet(packet)
        self._refresh_images()

        assert self.dataset is not None
        self.packet_position_label.setText(
            f"Packet {self.packet_index + 1} of {len(self.dataset)}  ·  "
            f"{packet.metadata.get('packet_id', 'unknown')}"
        )
        self.previous_button.setEnabled(self.packet_index > 0)
        self.next_button.setEnabled(self.packet_index + 1 < len(self.dataset))

    def _refresh_images(self) -> None:
        if self.packet is None or self.dataset is None:
            return
        cropped = self.crop_checkbox.isChecked()
        rgb_polygon = self.packet.geometry.get("rgb_patch_polygon_pixels")
        depth_polygon = self.packet.geometry.get("depth_patch_polygon_pixels")
        self.rgb_canvas.show_rgb(
            self.packet.rgb,
            self.packet.rgb_mask,
            rgb_polygon,
            cropped=cropped,
        )

        depth_m = self.dataset.depth_to_metres(self.packet.depth)
        limits = self.fixed_depth_limits_m
        if self.auto_depth_checkbox.isChecked():
            valid = depth_m[np.isfinite(depth_m) & (depth_m > 0.0)]
            if valid.size:
                lower = float(np.min(valid))
                upper = float(np.max(valid))
                if lower < upper:
                    limits = (lower, upper)
        self.depth_canvas.show_depth(
            depth_m,
            self.packet.depth_mask,
            depth_polygon,
            limits,
            cropped=cropped,
        )

    def previous_packet(self) -> None:
        self.show_packet(self.packet_index - 1)

    def next_packet(self) -> None:
        self.show_packet(self.packet_index + 1)
