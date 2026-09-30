"""Command-line entry points for the offline dataset pipeline."""

import argparse
from pathlib import Path

from legwheel_dataset.config import ProcessingConfig
from legwheel_dataset.processor import extract_features, isolate_packets


def _stage1_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--run-directory", type=Path, required=True)
    parser.add_argument("--experiment-config", type=Path)
    parser.add_argument("--processing-config", type=Path)
    parser.add_argument("--output-directory", type=Path)
    return parser


def isolate_packets_main(args=None) -> None:
    """Run only Stage 1 packet isolation."""
    parser = _stage1_parser("Isolate synchronized learning packets from MCAP.")
    options = parser.parse_args(args)
    config = ProcessingConfig.from_yaml(options.processing_config)
    output = isolate_packets(
        options.run_directory,
        config,
        options.experiment_config,
        options.output_directory,
    )
    print(f"Stage 1 dataset written to {output}")


def extract_features_main(args=None) -> None:
    """Run only Stage 2 feature extraction."""
    parser = argparse.ArgumentParser(
        description="Extract ROI, plane, and spectral features."
    )
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--processing-config", type=Path)
    options = parser.parse_args(args)
    config = (
        ProcessingConfig.from_yaml(options.processing_config)
        if options.processing_config
        else None
    )
    output = extract_features(options.dataset, config)
    print(f"Stage 2 features written to {output}")


def build_dataset_main(args=None) -> None:
    """Run Stage 1 followed immediately by Stage 2."""
    parser = _stage1_parser("Build the complete LegWheel ML dataset.")
    options = parser.parse_args(args)
    config = ProcessingConfig.from_yaml(options.processing_config)
    output = isolate_packets(
        options.run_directory,
        config,
        options.experiment_config,
        options.output_directory,
    )
    extract_features(output, config)
    print(f"Complete dataset written to {output}")
