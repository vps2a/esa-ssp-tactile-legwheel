"""Interactive creator for a run's ``postprocess.yaml`` file."""

import argparse
from dataclasses import fields, replace
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Callable

from legwheel_dataset.config import ProcessingConfig


SCHEMA_VERSION = "2.0"
CONFIG_FILENAME = "postprocess.yaml"
MAXIMUM_PREVIOUS_RUN_DIRECTORIES = 3

# The first sentence is shown in the generated YAML. Both sentences are shown
# when the user enters -h or --help at an interactive prompt.
FIELD_HELP = {
    "rgb_depth_max_delta_ms": (
        "Maximum RGB/depth timestamp separation in milliseconds.",
        "Smaller values improve temporal alignment but can reject more pairs.",
    ),
    "spectral_window_ms": (
        "Duration of telemetry used for each FFT in milliseconds.",
        "Longer windows improve frequency resolution but need more valid data.",
    ),
    "telemetry_resample_hz": (
        "Common sampling frequency for all 17 telemetry channels in hertz.",
        "It sets the FFT Nyquist frequency and must respect measured rates.",
    ),
    "maximum_interpolation_gap_ms": (
        "Largest source-data gap that interpolation may bridge.",
        "Larger values retain more samples but may conceal telemetry dropouts.",
    ),
    "maximum_packet_saving_frequency_hz": (
        "Maximum rate at which synchronized packets may be saved.",
        "It limits near-duplicate packets; rejected candidates do not use a slot.",
    ),
    "alpha_off_rad": (
        "Positive angular distance from the camera to the patch centre.",
        "The measured direction of travel is applied automatically.",
    ),
    "alpha_sp_rad": (
        "Full angular width of the selected terrain patch.",
        "It controls patch size and the calculated local telemetry duration.",
    ),
    "grouser_effective_height_m": (
        "Effective grouser height added to wheel radius in metres.",
        "It changes wheel-motion prediction before encoder correction.",
    ),
    "minimum_target_torque_fraction": (
        "Minimum fraction of requested torque defining the steady run.",
        "Higher values exclude more of the torque ramp from processing.",
    ),
    "source_rate_tolerance_fraction": (
        "Allowed fractional margin above each measured telemetry rate.",
        "It controls when an overly high resampling rate is rejected.",
    ),
    "track_point_count": (
        "Number of samples used around each full circular track boundary.",
        "More points smooth overlays but increase projection work and file size.",
    ),
    "depth_scale_m": (
        "Metres represented by one integer depth-image unit.",
        "An incorrect scale changes every reconstructed 3D point and plane.",
    ),
    "minimum_plane_points": (
        "Minimum valid depth points and RANSAC inliers required for a plane.",
        "Higher values demand stronger support but reject sparse patches.",
    ),
    "plane_ransac_iterations": (
        "Number of random plane candidates tested by RANSAC.",
        "More iterations improve robustness but take longer to process.",
    ),
    "plane_inlier_threshold_m": (
        "Maximum point-to-plane distance for a RANSAC inlier in metres.",
        "Smaller values reject more noise but may reject rough terrain.",
    ),
    "minimum_spectral_valid_fraction": (
        "Minimum valid fraction of an FFT channel before spectrum calculation.",
        "Higher values reject spectra containing more missing telemetry.",
    ),
}

InputFunction = Callable[[str], str]
OutputFunction = Callable[[str], None]


def _editable_field_names() -> list[str]:
    """Return dataclass fields in the stable order used by the YAML file."""
    return [field.name for field in fields(ProcessingConfig)
            if field.name != "schema_version"]


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    """Load a candidate source without requiring every field to be present."""
    import yaml

    with path.open("r", encoding="utf-8") as file:
        values = yaml.safe_load(file)
    if not isinstance(values, dict):
        raise ValueError(f"Configuration must contain a YAML mapping: {path}")
    schema_version = values.get("schema_version", SCHEMA_VERSION)
    if schema_version != SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported schema_version {schema_version!r} in {path}; "
            f"expected {SCHEMA_VERSION!r}."
        )
    return values


def find_previous_config(run_directory: Path) -> Path | None:
    """Find a config in at most the three nearest earlier sibling runs."""
    match = re.fullmatch(r"run_(\d+)", run_directory.name)
    if match is None:
        return None
    current_number = int(match.group(1))
    candidates = []
    for sibling in run_directory.parent.iterdir():
        sibling_match = re.fullmatch(r"run_(\d+)", sibling.name)
        if not sibling.is_dir() or sibling_match is None:
            continue
        run_number = int(sibling_match.group(1))
        if run_number < current_number:
            candidates.append((run_number, sibling))
    candidates.sort(key=lambda item: item[0], reverse=True)
    for _, sibling in candidates[:MAXIMUM_PREVIOUS_RUN_DIRECTORIES]:
        candidate = sibling / CONFIG_FILENAME
        if candidate.is_file():
            return candidate
    return None


def _coerce_value(field_name: str, value: Any) -> int | float:
    """Convert YAML or terminal input to the field's required numeric type."""
    default = getattr(ProcessingConfig(), field_name)
    if isinstance(value, bool):
        raise ValueError("Boolean values are not accepted here.")
    if isinstance(default, int):
        if isinstance(value, int):
            return value
        if isinstance(value, str):
            stripped = value.strip()
            if re.fullmatch(r"[+-]?\d+", stripped):
                return int(stripped)
        raise ValueError("Enter a whole number.")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("Enter a number.") from error
    return result


def _validated_field_value(field_name: str, value: Any) -> int | float:
    """Coerce one value and apply the same validation as the processor."""
    coerced = _coerce_value(field_name, value)
    candidate = replace(ProcessingConfig(), **{field_name: coerced})
    candidate.validate()
    return coerced


def resolve_initial_values(
    run_directory: Path,
    output: OutputFunction = print,
) -> tuple[dict[str, int | float], dict[str, str], Path | None]:
    """Resolve current-file, previous-run, then built-in defaults per field."""
    current_path = run_directory / CONFIG_FILENAME
    previous_path = find_previous_config(run_directory)
    sources: list[tuple[str, Path]] = []
    if current_path.is_file():
        sources.append(("current file", current_path))
    if previous_path is not None:
        sources.append((f"previous run ({previous_path.parent.name})", previous_path))

    loaded_sources = []
    for label, path in sources:
        try:
            loaded_sources.append((label, path, _load_yaml_mapping(path)))
        except (OSError, ValueError) as error:
            output(f"Warning: ignoring {label} {path}: {error}")

    defaults = ProcessingConfig()
    values: dict[str, int | float] = {}
    origins: dict[str, str] = {}
    for field_name in _editable_field_names():
        selected = False
        for label, path, mapping in loaded_sources:
            if field_name not in mapping:
                continue
            try:
                values[field_name] = _validated_field_value(
                    field_name, mapping[field_name]
                )
            except ValueError as error:
                output(
                    f"Warning: ignoring invalid {field_name} in {path}: {error}"
                )
                continue
            origins[field_name] = label
            selected = True
            break
        if not selected:
            values[field_name] = getattr(defaults, field_name)
            origins[field_name] = "built-in default"
    return values, origins, previous_path


def _is_help(response: str) -> bool:
    return response.strip().lower() in {"-h", "--help"}


def _ask_yes_no(
    prompt: str,
    default: bool,
    input_function: InputFunction,
    output: OutputFunction,
    help_text: str | None = None,
) -> bool:
    """Ask a repeatable yes/no question with optional interactive help."""
    while True:
        response = input_function(prompt).strip().lower()
        if _is_help(response) and help_text is not None:
            output(help_text)
            continue
        if not response:
            return default
        if response in {"y", "yes"}:
            return True
        if response in {"n", "no"}:
            return False
        suffix = ", -h for help" if help_text is not None else ""
        output(f"Please enter y or n{suffix}.")


def prompt_for_values(
    initial_values: dict[str, int | float],
    origins: dict[str, str],
    input_function: InputFunction = input,
    output: OutputFunction = print,
) -> ProcessingConfig:
    """Walk through every editable value and return a validated config."""
    selected_values = dict(initial_values)
    for field_name in _editable_field_names():
        description, influence = FIELD_HELP[field_name]
        output("")
        output(field_name)
        output(f"Current value: {selected_values[field_name]} ({origins[field_name]})")
        change = _ask_yes_no(
            "Change this value? [y/N, -h for help]: ",
            False,
            input_function,
            output,
            f"{description} {influence}",
        )
        if not change:
            continue
        while True:
            response = input_function(
                "New value [-h for help, Enter to keep current]: "
            ).strip()
            if _is_help(response):
                output(f"{description} {influence}")
                continue
            if not response:
                break
            try:
                selected_values[field_name] = _validated_field_value(
                    field_name, response
                )
                break
            except ValueError as error:
                output(f"Invalid value: {error}")

    config = ProcessingConfig(**selected_values)
    config.validate()
    return config


def render_processing_config(
    config: ProcessingConfig,
    created_utc: str,
) -> str:
    """Render stable, readable YAML with one short comment per setting."""
    lines = [
        "# LegWheel offline dataset post-processing configuration.",
        f'schema_version: "{config.schema_version}"',
        f'created_utc: "{created_utc}"',
    ]
    for field_name in _editable_field_names():
        lines.append(f"# {FIELD_HELP[field_name][0]}")
        lines.append(f"{field_name}: {getattr(config, field_name)!r}")
    return "\n".join(lines) + "\n"


def write_processing_config(path: Path, contents: str) -> None:
    """Atomically create or replace a configuration in its run directory."""
    temporary_path = None
    output_mode = (path.stat().st_mode & 0o777) if path.exists() else 0o644
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_file.write(contents)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
            temporary_path = Path(temporary_file.name)
        os.chmod(temporary_path, output_mode)
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def _print_summary(config: ProcessingConfig, output: OutputFunction) -> None:
    output("\nFinal algorithm settings:")
    for name, value in config.to_dict().items():
        output(f"  {name}: {value}")


def create_config(
    run_directory: Path,
    input_function: InputFunction = input,
    output: OutputFunction = print,
) -> Path | None:
    """Run the interactive workflow and return the saved path, if any."""
    run_directory = run_directory.expanduser().resolve()
    if not run_directory.is_dir():
        raise FileNotFoundError(f"Run directory does not exist: {run_directory}")

    config_path = run_directory / CONFIG_FILENAME
    initial_values, origins, previous_path = resolve_initial_values(
        run_directory, output
    )
    if config_path.is_file():
        output(f"Using existing values from {config_path} where available.")
    if previous_path is not None:
        output(f"Previous-run fallback: {previous_path}")
    else:
        output("No postprocess.yaml was found in the three nearest earlier runs.")

    config = prompt_for_values(
        initial_values, origins, input_function, output
    )
    _print_summary(config, output)
    overwrite = config_path.exists()
    save_prompt = (
        f"Replace {config_path} atomically? [y/N]: "
        if overwrite
        else f"Save to {config_path}? [Y/n]: "
    )
    if not _ask_yes_no(
        save_prompt, not overwrite, input_function, output
    ):
        output("Configuration was not saved.")
        return None

    created_utc = datetime.now(timezone.utc).isoformat(timespec="seconds")
    write_processing_config(
        config_path, render_processing_config(config, created_utc)
    )
    # Re-open with the production loader before claiming success.
    saved_config = ProcessingConfig.from_yaml(config_path)
    output(f"Saved and verified {config_path}")

    if _ask_yes_no(
        "Run validate_config against this recording now? [y/N]: ",
        False,
        input_function,
        output,
    ):
        from legwheel_dataset.processor import validate_processing_configuration

        report = validate_processing_configuration(
            run_directory, saved_config
        )
        output(json.dumps(report, indent=2))
    return config_path


def main(args=None) -> None:
    """ROS 2 console-script entry point."""
    parser = argparse.ArgumentParser(
        description=(
            "Interactively create RUN_DIRECTORY/postprocess.yaml using current, "
            "previous-run, or built-in values."
        )
    )
    parser.add_argument("--run-directory", type=Path, required=True)
    options = parser.parse_args(args)
    create_config(options.run_directory)
