import math
import unittest

import numpy as np

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.projection import CameraCalibration
from legwheel_kinematics.track import project_track
from legwheel_kinematics.transforms import calculate_theta_p, forward_kinematics


def notebook_config() -> KinematicsConfig:
    return KinematicsConfig(
        d1=0.353,
        a_b=0.83,
        h_b=0.0761,
        camera_beam_offset=-0.043,
        camera_height=0.029,
        camera_offset_from_beam_centre=0.04,
        camera_angle_deg=40.0,
        Beam_radius=0.83,
        Wheel_width=0.1,
        wheel_radius_mm=100.0,
        hip_link_length_m=0.2,
        calf_link_length_m=0.2,
    )


class KinematicsTest(unittest.TestCase):
    def test_experiment_mapping_uses_measured_constants(self):
        config = KinematicsConfig.from_experiment_dict({
            "rig_config": {"pivot_height_m": 0.353, "beam_radius_m": 0.83},
            "leg_config": {
                "hip_link_length_m": 0.2,
                "calf_link_length_m": 0.21,
            },
            "electronics_hardware": {
                "camera": {
                    "camera_config": {
                        "camera_beam_offset_m": -0.043,
                        "camera_height_m": 0.029,
                        "camera_front_angle_deg": 40.0,
                    }
                }
            },
            "wheel_config": {"wheel_radius_mm": 100.0, "wheel_width_mm": 100.0},
        })
        self.assertEqual(config.d1, 0.353)
        self.assertEqual(config.h_b, 0.0761)
        self.assertEqual(config.camera_offset_from_beam_centre, 0.04)

    def test_forward_kinematics_matches_notebook_reference(self):
        expected = np.array([
            [-1.0, 0.0, 0.0, 0.787],
            [0.0, 0.64278761, -0.76604444, -0.04],
            [0.0, -0.76604444, -0.64278761, 0.4581],
            [0.0, 0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(
            forward_kinematics(notebook_config(), theta_p=0.0),
            expected,
            atol=1e-8,
        )

    def test_theta_p_uses_supplied_rig_equation(self):
        knee_joint = 1.0
        expected_argument = (
            100.0 / 1000.0
            + 0.085
            + math.sqrt(0.2**2 + 0.2**2 - 2 * 0.2 * 0.2 * math.cos(knee_joint))
            - 0.0761
            - 0.353
        )
        self.assertAlmostEqual(
            calculate_theta_p(knee_joint, 100.0, 0.2, 0.2, 0.353),
            math.asin(expected_argument),
        )

    def test_theta_p_rejects_impossible_arcsin_input(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            calculate_theta_p(0.0, 2000.0, 0.2, 0.2, 0.1)

    def test_projected_track_forms_visible_polygon(self):
        calibration = CameraCalibration(
            width=640,
            height=480,
            K=np.array([
                [461.8016663, 0.0, 324.1998901],
                [0.0, 461.7047424, 242.8716125],
                [0.0, 0.0, 1.0],
            ]),
            D=np.array([0.0056061, -0.0481728, -0.00040035, 0.00041010, 0.0330434]),
            distortion_model="plumb_bob",
        )
        projection = project_track(
            notebook_config(),
            theta_p=0.0,
            calibration=calibration,
            point_count=360,
        )
        self.assertGreater(projection.visible_indices.size, 0)
        self.assertEqual(projection.polygon_pixels.shape[1], 2)
        self.assertTrue(np.all(np.isfinite(projection.polygon_pixels)))


if __name__ == "__main__":
    unittest.main()
