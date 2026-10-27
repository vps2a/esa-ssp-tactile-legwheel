import math
from dataclasses import replace
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.projection import CameraCalibration
from legwheel_kinematics.track import generate_track_points, project_track
from legwheel_kinematics.transforms import (
    forward_kinematics,
    frame_coordinates,
    rotation_matrix_to_quaternion,
)


def notebook_config() -> KinematicsConfig:
    return KinematicsConfig(
        d1=0.353,
        a_b=0.83,
        h_b=0.0761,
        sensor_front_offset=0.043,
        camera_height=0.029,
        camera_beam_offset=0.054,
        camera_angle_deg=40.0,
        Beam_radius=0.83,
        Wheel_width=0.1,
        wheel_radius_mm=100.0,
        hip_link_length_m=0.2,
        calf_link_length_m=0.2,
        theta_y_kinematic_sign=-1.0,
    )


class KinematicsTest(unittest.TestCase):
    def test_experiment_mapping_uses_explicit_camera_offsets(self):
        config = KinematicsConfig.from_experiment_dict({
            "rig_config": {"pivot_height_m": 0.353, "beam_radius_m": 0.83},
            "leg_config": {
                "hip_link_length_m": 0.2,
                "calf_link_length_m": 0.21,
            },
            "electronics_hardware": {
                "encoder_setup": {"theta_y_kinematic_sign": -1},
                "camera": {
                    "camera_config": {
                        "sensor_front_offset_m": 0.039,
                        "camera_beam_offset_m": 0.062,
                        "camera_height_m": 0.029,
                        "camera_front_angle_deg": 40.0,
                    }
                }
            },
            "wheel_config": {"wheel_radius_mm": 100.0, "wheel_width_mm": 100.0},
        })
        self.assertEqual(config.d1, 0.353)
        self.assertEqual(config.h_b, 0.0661)
        self.assertEqual(config.sensor_front_offset, 0.039)
        self.assertEqual(config.camera_beam_offset, 0.062)
        self.assertAlmostEqual(config.track_inner_radius_m, 0.73)
        self.assertAlmostEqual(config.track_outer_radius_m, 0.83)
        self.assertAlmostEqual(config.track_center_radius_m, 0.78)
        self.assertEqual(config.theta_y_kinematic_sign, -1.0)

    def test_track_width_must_leave_a_positive_inner_radius(self):
        for width in (0.0, -0.1, 0.83, 0.9, math.inf, math.nan):
            with self.subTest(width=width):
                with self.assertRaisesRegex(ValueError, "Wheel width"):
                    replace(notebook_config(), Wheel_width=width)

    def test_motion_angles_are_converted_to_notebook_convention(self):
        config = notebook_config()
        self.assertEqual(config.theta_y_to_kinematic(1.25), -1.25)
        self.assertEqual(config.direction_to_kinematic(1.0), -1.0)
        self.assertTrue(config.is_forward_motion(1.0))
        self.assertFalse(config.is_forward_motion(-1.0))

    def test_kinematic_sign_must_be_exactly_plus_or_minus_one(self):
        experiment_config = {
            "rig_config": {"pivot_height_m": 0.353, "beam_radius_m": 0.83},
            "leg_config": {
                "hip_link_length_m": 0.2,
                "calf_link_length_m": 0.21,
            },
            "electronics_hardware": {
                "encoder_setup": {"theta_y_kinematic_sign": 0},
                "camera": {
                    "camera_config": {
                        "sensor_front_offset_m": 0.043,
                        "camera_beam_offset_m": 0.054,
                        "camera_height_m": 0.029,
                        "camera_front_angle_deg": 40.0,
                    }
                },
            },
            "wheel_config": {
                "wheel_radius_mm": 100.0,
                "wheel_width_mm": 100.0,
            },
        }
        with self.assertRaisesRegex(ValueError, "must be exactly -1 or 1"):
            KinematicsConfig.from_experiment_dict(experiment_config)

    def test_forward_kinematics_matches_notebook_reference(self):
        expected = np.array([
            [-1.0, 0.0, 0.0, 0.787],
            [0.0, 0.64278761, -0.76604444, -0.054],
            [0.0, -0.76604444, -0.64278761, 0.4581],
            [0.0, 0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(
            forward_kinematics(notebook_config(), theta_p=0.0),
            expected,
            atol=1e-8,
        )

    def test_frame_coordinates_match_notebook_chain(self):
        Frame_coords = frame_coordinates(notebook_config(), theta_p=0.0)
        self.assertEqual(len(Frame_coords), 6)
        np.testing.assert_array_equal(Frame_coords[0], np.eye(4))
        np.testing.assert_allclose(
            Frame_coords[-1],
            forward_kinematics(notebook_config(), theta_p=0.0),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            np.vstack([frame[:3, 3] for frame in Frame_coords]),
            np.array([
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.353],
                [0.0, 0.0, 0.353],
                [0.83, 0.0, 0.4291],
                [0.787, -0.054, 0.4581],
                [0.787, -0.054, 0.4581],
            ]),
            atol=1e-12,
        )

    def test_camera_offsets_follow_beam_axes_without_rotating_camera(self):
        reference = replace(
            notebook_config(), sensor_front_offset=0.0, camera_beam_offset=0.0
        )
        for yaw in (0.0, 0.7):
            for pitch in (0.0, -0.2):
                original = frame_coordinates(reference, pitch, yaw)
                beam_axes = original[3][:3, :3]
                for offset in (-0.01, 0.02):
                    cases = (
                        (
                            replace(reference, sensor_front_offset=offset),
                            -offset * beam_axes[:, 0],
                        ),
                        (
                            replace(reference, camera_beam_offset=offset),
                            offset * beam_axes[:, 2],
                        ),
                    )
                    for geometry, expected_displacement in cases:
                        with self.subTest(yaw=yaw, pitch=pitch, geometry=geometry):
                            shifted = frame_coordinates(geometry, pitch, yaw)
                            np.testing.assert_allclose(
                                shifted[-1][:3, 3] - original[-1][:3, 3],
                                expected_displacement,
                                atol=1e-12,
                            )
                            np.testing.assert_array_equal(shifted[3], original[3])
                            np.testing.assert_array_equal(
                                shifted[-1][:3, :3], original[-1][:3, :3]
                            )

    def test_robot_description_uses_the_same_signed_camera_offsets(self):
        try:
            import xacro
        except ImportError:
            self.skipTest("Install xacro to check the robot description.")
        path = (
            Path(__file__).resolve().parents[2]
            / "legwheel_description/urdf/legwheel.urdf.xacro"
        )
        for front, side in ((0.043, 0.054), (0.0, 0.0), (-0.01, -0.02)):
            with self.subTest(front=front, side=side):
                config = replace(
                    notebook_config(), sensor_front_offset=front, camera_beam_offset=side
                )
                mappings = {
                    name: str(getattr(config, name))
                    for name in (
                        "d1", "a_b", "h_b",
                        "sensor_front_offset", "camera_height", "camera_beam_offset",
                    )
                }
                mappings["camera_angle_rad"] = str(
                    math.radians(config.camera_angle_deg)
                )
                robot = ET.fromstring(
                    xacro.process_file(str(path), mappings=mappings).toxml()
                )
                origin = robot.find("joint[@name='camera_mount_translation']/origin")
                np.testing.assert_allclose(
                    [float(value) for value in origin.attrib["xyz"].split()],
                    [-front, config.camera_height, side],
                    atol=1e-12,
                )

    def test_rotation_matrix_to_quaternion_preserves_rotation(self):
        rotation = forward_kinematics(
            notebook_config(),
            theta_p=0.2,
        )[:3, :3]
        x, y, z, w = rotation_matrix_to_quaternion(rotation)
        recovered = np.array([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ])
        np.testing.assert_allclose(recovered, rotation, atol=1e-12)
        self.assertAlmostEqual(x * x + y * y + z * z + w * w, 1.0)

    def test_frame_coordinates_apply_measured_yaw_in_stationary_base(self):
        config = notebook_config()
        yaw = 0.7
        rotation = np.array([
            [math.cos(yaw), -math.sin(yaw), 0, 0],
            [math.sin(yaw), math.cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        original = frame_coordinates(config, theta_p=0.2)
        rotated = frame_coordinates(config, theta_p=0.2, theta_y=yaw)
        np.testing.assert_array_equal(rotated[0], np.eye(4))
        for fixed_pose, yaw_pose in zip(original[1:], rotated[1:]):
            np.testing.assert_allclose(yaw_pose, rotation @ fixed_pose, atol=1e-12)

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

    def test_track_points_use_configured_inner_and_outer_radii(self):
        points = generate_track_points(notebook_config(), point_count=360)
        inner_radii = np.linalg.norm(points.inner_track_points[:, :2], axis=1)
        outer_radii = np.linalg.norm(points.outer_track_points[:, :2], axis=1)
        np.testing.assert_allclose(inner_radii, 0.73, atol=1e-12)
        np.testing.assert_allclose(outer_radii, 0.83, atol=1e-12)
        np.testing.assert_array_equal(
            points.inner_track_points[:, 2],
            np.zeros(360),
        )


if __name__ == "__main__":
    unittest.main()
