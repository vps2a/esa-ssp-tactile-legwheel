from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import unittest
from unittest.mock import patch

from legwheel_dataset.cli import validate_config_main
from legwheel_dataset.config import ProcessingConfig


class ValidationCliTest(unittest.TestCase):
    def test_without_show_flag_uses_report_only_path(self):
        report = {"visualization_preview": {"available": True}}
        stdout = StringIO()
        with (
            patch.object(
                ProcessingConfig,
                "from_yaml",
                return_value=ProcessingConfig(),
            ),
            patch(
                "legwheel_dataset.cli.validate_processing_configuration",
                return_value=report,
            ) as validate_report_only,
            patch(
                "legwheel_dataset.cli."
                "validate_processing_configuration_with_preview",
            ) as validate_with_preview,
            redirect_stdout(stdout),
        ):
            validate_config_main([
                "--run-directory",
                "/recording/run_1",
            ])

        self.assertIn('"available": true', stdout.getvalue())
        validate_report_only.assert_called_once()
        validate_with_preview.assert_not_called()

    def test_show_flag_prints_json_before_requesting_plot(self):
        report = {"visualization_preview": {"available": True}}
        preview = object()
        events = []

        def show(selected_preview):
            self.assertIs(selected_preview, preview)
            events.append("show")
            return True, None

        stdout = StringIO()
        with (
            patch.object(
                ProcessingConfig,
                "from_yaml",
                return_value=ProcessingConfig(),
            ),
            patch(
                "legwheel_dataset.cli."
                "validate_processing_configuration_with_preview",
                return_value=(report, preview),
            ),
            patch(
                "legwheel_dataset.validation_plot.show_validation_preview",
                side_effect=show,
            ),
            redirect_stdout(stdout),
        ):
            validate_config_main([
                "--run-directory",
                "/recording/run_1",
                "--show-visualization",
            ])

        self.assertIn('"available": true', stdout.getvalue())
        self.assertEqual(events, ["show"])

    def test_missing_preview_prints_yellow_warning_to_stderr(self):
        report = {
            "visualization_preview": {
                "available": False,
                "reason": "no_synchronized_pair",
            }
        }
        stdout = StringIO()
        stderr = StringIO()
        with (
            patch.object(
                ProcessingConfig,
                "from_yaml",
                return_value=ProcessingConfig(),
            ),
            patch(
                "legwheel_dataset.cli."
                "validate_processing_configuration_with_preview",
                return_value=(report, None),
            ),
            redirect_stdout(stdout),
            redirect_stderr(stderr),
        ):
            validate_config_main([
                "--run-directory",
                "/recording/run_1",
                "--show-visualization",
            ])

        self.assertIn('"available": false', stdout.getvalue())
        self.assertIn("\033[33mWarning:", stderr.getvalue())
        self.assertIn("no_synchronized_pair", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
