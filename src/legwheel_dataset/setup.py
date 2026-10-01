from glob import glob
import os

from setuptools import find_packages, setup


package_name = "legwheel_dataset"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Filip Szczebak",
    maintainer_email="f.szczebak@gmail.com",
    description="Offline MCAP to machine-learning dataset processing.",
    license="TODO",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "isolate_packets = legwheel_dataset.cli:isolate_packets_main",
            "extract_features = legwheel_dataset.cli:extract_features_main",
            "build_dataset = legwheel_dataset.cli:build_dataset_main",
            "validate_config = legwheel_dataset.cli:validate_config_main",
            "visualise_stream = legwheel_dataset.visualization:main",
        ],
    },
)
