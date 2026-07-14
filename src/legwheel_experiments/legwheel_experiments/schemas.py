from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

#Class representing the experiment configuration schema
@dataclass(frozen=True) #To make sure we don't overwrite the configuration accidentally
class ExperimentConfig:
    schema_version: str
    experiment_id: str
    datetime_of_creation: str
    rig_config: dict[str, Any]
    wheel_leg_config: dict[str, Any]
    electronics_hardware: dict[str, Any]
    wheel_config: dict[str, Any]
    environment: dict[str, Any]

#Class representing the run configuration schema
@dataclass(frozen=True)
class RunConfig:
    schema_version: str
    experiment_id: str
    run_id: str
    datetime_of_creation: str
    run_length_loops: int
    leg_parameters: dict[str, Any]
    wheel_parameters: dict[str, Any]
    data_aquistion_rates: dict[str, Any]

REQUIRED_EXP_CONFIG_SECTIONS = [
    "schema_version",
    "experiment_id",
    "datetime_of_creation",
    "rig_config",
    "leg_config",
    "electronics_hardware",
    "wheel_config",
    "environment",
]

REQUIRED_RUN_CONFIG_SECTIONS = [
    "schema_version"
    "experiment_id",
    "run_id",
    "datetime_of_creation",
    "run_length_loops",
    "leg_parameters",
    "wheel_parameters",
    "data_aquistion_rates",
]

def load_experiment_config(config_path: Path) -> ExperimentConfig:
    """Load experiment configuration from a YAML file."""
    with config_path.open("r", encoding="utf-8") as file:
        raw_config = yaml.safe_load(file)

    if not isinstance(raw_config, dict):
            raise ValueError("Experiment configuration must contain a YAML mapping.")

    return ExperimentConfig(
        schema_version=raw_config["schema_version"],
        experiment_id=raw_config["experiment_id"],
        datetime_of_creation=raw_config["datetime_of_creation"],
        rig_config=raw_config["rig_config"],
        wheel_leg_config=raw_config["leg_config"],
        electronics_hardware=raw_config["electronics_hardware"],
        wheel_config=raw_config["wheel_config"],
        environment=raw_config["environment"],
    )

#TODO: Add load run config function

def require_positive_number(value: Any, field_name: str) -> None:
    if value is None:
        raise ValueError(f"{field_name} has not been configured.")

    if not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a number.")

    if value <= 0:
        raise ValueError(f"{field_name} must be greater than zero.")
    
#TODO: Add more check types then just positive number
        
def validate_structure(raw_config: dict[str, Any]) -> None:
    """Validate the structure of the experiment configuration."""
    for section in REQUIRED_EXP_CONFIG_SECTIONS:
        if section not in raw_config:
            raise ValueError(f"Missing required section: {section}")
        
        if not isinstance(raw_config[section], dict):
            raise ValueError(f"Section '{section}' must be a YAML mapping.")
        
def validate_experiment_config(config: ExperimentConfig) -> None:
    """Validate the experiment configuration."""
    # Validate rig_config
    rig_config = config.rig_config
    require_positive_number(rig_config.get("beam_radius_m"), "beam_radius_m")
    require_positive_number(rig_config.get("beam_total_length_m"), "beam_total_length_m")
    
    # Validate eg_config
    leg_config = config.leg_config
    require_positive_number(leg_config.get("hip_link_length_m"), "hip_link_length_m")
    require_positive_number(leg_config.get("calf_link_length_m"), "calf_link_length_m")
    
    #TODO:Validate electronics_hardware
    
    #TODO:Validate wheel_config
    