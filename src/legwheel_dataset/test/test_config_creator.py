from pathlib import Path
import tempfile
import unittest

from legwheel_dataset.config import ProcessingConfig
from legwheel_dataset.config_creator import (
    create_config,
    find_previous_config,
    prompt_for_values,
    render_processing_config,
    resolve_initial_values,
    write_processing_config,
)


class ConfigLoaderTest(unittest.TestCase):
    def test_loader_accepts_creation_metadata_but_excludes_it_from_values(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "postprocess.yaml"
            path.write_text(
                'schema_version: "2.0"\n'
                'created_utc: "2026-10-01T12:00:00+00:00"\n'
                "rgb_depth_max_delta_ms: 7.5\n",
                encoding="utf-8",
            )
            config = ProcessingConfig.from_yaml(path)

        self.assertEqual(config.rgb_depth_max_delta_ms, 7.5)
        self.assertNotIn("created_utc", config.to_dict())

    def test_loader_rejects_creation_time_without_timezone(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "postprocess.yaml"
            path.write_text(
                'schema_version: "2.0"\n'
                'created_utc: "2026-10-01T12:00:00"\n',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "timezone"):
                ProcessingConfig.from_yaml(path)


class ConfigCreatorTest(unittest.TestCase):
    def test_nearest_three_previous_runs_are_searched_newest_first(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for number in range(1, 6):
                (root / f"run_{number}").mkdir()
            (root / "run_3" / "postprocess.yaml").write_text(
                "alpha_sp_rad: 0.2\n", encoding="utf-8"
            )
            (root / "run_1" / "postprocess.yaml").write_text(
                "alpha_sp_rad: 0.3\n", encoding="utf-8"
            )

            found = find_previous_config(root / "run_5")

        self.assertEqual(found, root / "run_3" / "postprocess.yaml")

    def test_previous_search_does_not_go_beyond_three_siblings(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for number in range(1, 6):
                (root / f"run_{number}").mkdir()
            (root / "run_1" / "postprocess.yaml").write_text(
                "alpha_sp_rad: 0.3\n", encoding="utf-8"
            )

            found = find_previous_config(root / "run_5")

        self.assertIsNone(found)

    def test_values_fall_back_field_by_field(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            previous_run = root / "run_1"
            current_run = root / "run_2"
            previous_run.mkdir()
            current_run.mkdir()
            (previous_run / "postprocess.yaml").write_text(
                'schema_version: "2.0"\n'
                "alpha_sp_rad: 0.2\n"
                "track_point_count: 720\n",
                encoding="utf-8",
            )
            (current_run / "postprocess.yaml").write_text(
                'schema_version: "2.0"\n'
                "track_point_count: 180\n",
                encoding="utf-8",
            )

            values, origins, previous_path = resolve_initial_values(
                current_run, output=lambda _message: None
            )

        self.assertEqual(previous_path, previous_run / "postprocess.yaml")
        self.assertEqual(values["track_point_count"], 180)
        self.assertEqual(origins["track_point_count"], "current file")
        self.assertEqual(values["alpha_sp_rad"], 0.2)
        self.assertIn("run_1", origins["alpha_sp_rad"])
        self.assertEqual(values["depth_scale_m"], 0.001)
        self.assertEqual(origins["depth_scale_m"], "built-in default")

    def test_help_works_at_both_prompts_and_invalid_input_retries(self):
        defaults = ProcessingConfig().to_dict()
        defaults.pop("schema_version")
        origins = {name: "built-in default" for name in defaults}
        responses = iter(
            ["-h", "y", "--help", "not-a-number", "5.0"]
            + ["n"] * (len(defaults) - 1)
        )
        output = []

        config = prompt_for_values(
            defaults,
            origins,
            input_function=lambda _prompt: next(responses),
            output=output.append,
        )

        self.assertEqual(config.rgb_depth_max_delta_ms, 5.0)
        self.assertGreaterEqual(
            sum("temporal alignment" in line for line in output), 2
        )
        self.assertTrue(any("Invalid value" in line for line in output))

    def test_rendered_file_has_comments_and_loads_after_atomic_write(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "postprocess.yaml"
            contents = render_processing_config(
                ProcessingConfig(), "2026-10-01T12:00:00+00:00"
            )
            write_processing_config(path, contents)
            loaded = ProcessingConfig.from_yaml(path)

            self.assertEqual(loaded, ProcessingConfig())
            self.assertIn("# Maximum RGB/depth", path.read_text(encoding="utf-8"))
            self.assertEqual(list(path.parent.glob(".postprocess.yaml.*.tmp")), [])

    def test_new_file_is_saved_after_summary_confirmation(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_directory = Path(temporary_directory) / "run_1"
            run_directory.mkdir()
            editable_count = len(ProcessingConfig().to_dict()) - 1
            responses = iter(["n"] * editable_count + ["", "n"])

            saved_path = create_config(
                run_directory,
                input_function=lambda _prompt: next(responses),
                output=lambda _message: None,
            )

            self.assertEqual(
                saved_path, run_directory.resolve() / "postprocess.yaml"
            )
            self.assertEqual(
                ProcessingConfig.from_yaml(saved_path), ProcessingConfig()
            )

    def test_existing_file_is_not_replaced_without_confirmation(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_directory = Path(temporary_directory) / "run_1"
            run_directory.mkdir()
            config_path = run_directory / "postprocess.yaml"
            original = (
                'schema_version: "2.0"\n'
                'created_utc: "2026-10-01T12:00:00+00:00"\n'
                "rgb_depth_max_delta_ms: 6.0\n"
            )
            config_path.write_text(original, encoding="utf-8")
            editable_count = len(ProcessingConfig().to_dict()) - 1
            responses = iter(["n"] * editable_count + [""])

            saved_path = create_config(
                run_directory,
                input_function=lambda _prompt: next(responses),
                output=lambda _message: None,
            )

            self.assertIsNone(saved_path)
            self.assertEqual(config_path.read_text(encoding="utf-8"), original)


if __name__ == "__main__":
    unittest.main()
