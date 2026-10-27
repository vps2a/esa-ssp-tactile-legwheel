"""Compatibility imports for the shared kinematics IMU estimator.

The estimator lives below the visualizers and dataset package in the dependency
tree. Keeping these public imports also lets older analysis scripts keep using
their original module path without maintaining a second implementation.
"""

from legwheel_kinematics.imu_pitch import (
    GRAVITY_M_S2,
    PITCH_ESTIMATOR_VERSION,
    PitchTrajectory,
    estimate_pitch,
)

__all__ = [
    "GRAVITY_M_S2",
    "PITCH_ESTIMATOR_VERSION",
    "PitchTrajectory",
    "estimate_pitch",
]
