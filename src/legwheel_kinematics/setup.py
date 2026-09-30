from glob import glob
import os

from setuptools import find_packages, setup


package_name = "legwheel_kinematics"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Filip Szczebak",
    maintainer_email="f.szczebak@gmail.com",
    description="LegWheel kinematics, camera projection, and debug overlays.",
    license="TODO",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "kinematic_joint_state_node = "
            "legwheel_kinematics.kinematic_joint_state_node:main",
            "track_overlay_node = "
            "legwheel_kinematics.track_overlay_node:main",
        ],
    },
)
