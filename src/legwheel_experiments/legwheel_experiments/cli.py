from pathlib import Path

from legwheel_experiments.schemas import load_experiment_config

def main():
    print("Hello from legwheel_experiments! (by cli)")
    
    config_path = Path("/home/parallels/Documents/Repositories/"
        "esa-ssp-tactile-legwheel/data/"
        "legwheel_experiment_runs/experiment_1/"
        "exp_1_config.yaml")
    experiment_config = load_experiment_config(config_path)

    print(f"Loaded experiment: {experiment_config.experiment_id}")
    print(f"Wheel radius: {experiment_config.wheel_config['wheel_radius_mm']} mm")
    
if __name__ == '__main__':
    main()