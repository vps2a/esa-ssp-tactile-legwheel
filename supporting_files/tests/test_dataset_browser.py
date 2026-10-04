import os
from pathlib import Path
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from legwheel_dataset_browser.model import (  # noqa: E402
    DatasetFormatError,
    LegWheelDataset,
    crop_to_mask,
)
from legwheel_dataset_browser.window import DatasetBrowserWindow  # noqa: E402
from tests.synthetic_dataset import (  # noqa: E402
    CHANNEL_NAMES,
    create_synthetic_dataset,
)


class DatasetModelTests(unittest.TestCase):
    def test_loads_manifest_chronologically_and_restores_packet_data(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = create_synthetic_dataset(Path(temporary) / "abc123")
            dataset = LegWheelDataset.open(root)

            self.assertEqual(len(dataset), 2)
            self.assertEqual(dataset.experiment_id, "synthetic_experiment")
            self.assertEqual(dataset.run_number, 4)
            self.assertEqual(dataset.records[0]["image_time_ns"], 2_000_000_000)

            packet = dataset.load_packet(0)
            self.assertEqual(packet.channel_names, CHANNEL_NAMES)
            self.assertEqual(packet.rgb_mask.shape, packet.rgb.shape[:2])
            self.assertTrue(packet.stage2_available)
            self.assertEqual(packet.stage2_arrays["spectral_power"].shape, (51, 15))
            self.assertEqual(packet.point_map["points_base_m"].shape, (4, 3))

    def test_stage1_only_packet_remains_loadable(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = create_synthetic_dataset(Path(temporary) / "abc123")
            packet = LegWheelDataset.open(root).load_packet(1)

            self.assertFalse(packet.stage2_available)
            self.assertIsNone(packet.features)
            self.assertEqual(packet.stage2_arrays, {})
            self.assertEqual(packet.point_map, {})

    def test_depth_limits_are_fixed_across_dataset_and_ignore_zero(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = create_synthetic_dataset(Path(temporary) / "abc123")
            dataset = LegWheelDataset.open(root)

            self.assertEqual(dataset.global_depth_limits_m(), (1.0, 2.0))

    def test_crop_returns_patch_and_full_image_origin(self):
        image = [[row * 10 + column for column in range(5)] for row in range(4)]
        import numpy as np

        image = np.asarray(image)
        mask = np.zeros((4, 5), dtype=bool)
        mask[1:3, 2:5] = True

        crop, crop_mask, origin = crop_to_mask(image, mask)

        self.assertEqual(crop.shape, (2, 3))
        self.assertTrue(crop_mask.all())
        self.assertEqual(origin, (2, 1))

    def test_rejects_directory_without_dataset_manifest(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(DatasetFormatError):
                LegWheelDataset.open(temporary)


class DatasetWindowSmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        QCoreApplication.setOrganizationName("LegWheel Browser Tests")
        QCoreApplication.setApplicationName("LegWheel Browser Tests")
        cls.application = QApplication.instance() or QApplication([])

    def test_window_loads_and_navigates_without_writing_dataset(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = create_synthetic_dataset(Path(temporary) / "abc123")
            manifest_before = (root / "manifest.jsonl").read_bytes()
            window = DatasetBrowserWindow(root)

            self.assertIsNotNone(window.dataset)
            self.assertEqual(window.packet_index, 0)
            self.assertFalse(window.previous_button.isEnabled())
            self.assertTrue(window.next_button.isEnabled())
            self.assertIn("4.250", window.slope_summary.along_label.text())

            window.next_packet()

            self.assertEqual(window.packet_index, 1)
            self.assertTrue(window.previous_button.isEnabled())
            self.assertFalse(window.next_button.isEnabled())
            self.assertIn("Unavailable", window.slope_summary.along_label.text())
            self.assertEqual((root / "manifest.jsonl").read_bytes(), manifest_before)
            window.close()

    def test_window_jumps_to_one_based_sorted_packet_position(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = create_synthetic_dataset(Path(temporary) / "abc123")
            window = DatasetBrowserWindow(root)

            # Clicking the button interprets 2 as the second item in the
            # model's already-sorted packet list, not as a stored packet ID.
            window.jump_packet_input.setText("2")
            window.jump_packet_button.click()
            self.assertEqual(window.packet_index, 1)
            self.assertEqual(window.jump_packet_input.text(), "")
            self.assertEqual(window.jump_packet_error_label.text(), "")

            # Pressing Enter uses exactly the same navigation path.
            window.jump_packet_input.setText("1")
            window.jump_packet_input.returnPressed.emit()
            self.assertEqual(window.packet_index, 0)
            self.assertEqual(window.jump_packet_input.text(), "")
            window.close()

    def test_invalid_packet_jump_keeps_current_packet_and_shows_inline_error(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = create_synthetic_dataset(Path(temporary) / "abc123")
            window = DatasetBrowserWindow(root)

            for invalid_value in ("", "0", "3", "not-a-number"):
                window.jump_packet_input.setText(invalid_value)
                window.jump_to_packet()
                self.assertEqual(window.packet_index, 0)
                self.assertNotEqual(window.jump_packet_error_label.text(), "")

            window.close()


if __name__ == "__main__":
    unittest.main()
