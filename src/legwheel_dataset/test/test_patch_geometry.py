"""Check that physical patches and every full-track overlay share boundaries."""

import math
import unittest

import numpy as np

from legwheel_dataset.patch_geometry import (
    project_full_track_boundaries,
    project_track_patch,
)
from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.projection import CameraCalibration
from legwheel_kinematics.track import generate_track_points, project_track


class PatchGeometryTest(unittest.TestCase):
    def test_patch_live_overlay_and_full_tracks_share_the_same_edges(self):
        config = KinematicsConfig(
            d1=0.353,
            a_b=0.83,
            h_b=0.0661,
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
        for point_count in (64, 128):
            for theta_y in (0.0, 0.4):
                for theta_p in (-0.15, 0.2):
                    for distortion in (np.zeros(5), np.array([
                        0.0056, -0.0482, -0.0004, 0.0004, 0.0330,
                    ])):
                        with self.subTest(
                            point_count=point_count,
                            theta_y=theta_y,
                            theta_p=theta_p,
                            distortion=distortion.tolist(),
                        ):
                            calibration = CameraCalibration(
                                width=640,
                                height=480,
                                K=np.array([
                                    [461.8, 0.0, 324.2],
                                    [0.0, 461.7, 242.9],
                                    [0.0, 0.0, 1.0],
                                ]),
                                D=distortion,
                                distortion_model="plumb_bob",
                            )
                            patch = project_track_patch(
                                config, theta_y, theta_p, -math.pi / 4.0,
                                math.pi / 4.0, calibration, point_count,
                            )
                            inner, inner_valid, outer, outer_valid = (
                                project_full_track_boundaries(
                                    config, theta_y, theta_p,
                                    calibration, point_count,
                                )
                            )
                            # This exact sector lies on the full-circle sample
                            # grid, so its edges must coincide pixel for pixel.
                            indices = np.arange(
                                5 * point_count // 16,
                                7 * point_count // 16 + 1,
                            )
                            np.testing.assert_allclose(
                                patch.inner_pixels, inner[indices], atol=1e-8,
                            )
                            np.testing.assert_allclose(
                                patch.outer_pixels, outer[indices], atol=1e-8,
                            )
                            np.testing.assert_array_equal(
                                patch.inner_valid, inner_valid[indices],
                            )
                            np.testing.assert_array_equal(
                                patch.outer_valid, outer_valid[indices],
                            )
                            for points, radius in (
                                (patch.inner_points_base_m, 0.73),
                                (patch.outer_points_base_m, 0.83),
                            ):
                                np.testing.assert_allclose(
                                    np.linalg.norm(points[:, :2], axis=1),
                                    radius, atol=1e-12,
                                )
                            if theta_y == 0.0:
                                live = project_track(
                                    config, theta_p, calibration, point_count,
                                )
                                np.testing.assert_allclose(
                                    live.inner_pixels, inner, atol=1e-8,
                                )
                                np.testing.assert_allclose(
                                    live.outer_pixels, outer, atol=1e-8,
                                )
                                np.testing.assert_array_equal(
                                    live.valid, inner_valid & outer_valid,
                                )
            track = generate_track_points(config, point_count)
            np.testing.assert_allclose(
                np.linalg.norm(track.inner_track_points[:, :2], axis=1), 0.73,
            )
            np.testing.assert_allclose(
                np.linalg.norm(track.outer_track_points[:, :2], axis=1), 0.83,
            )


if __name__ == "__main__":
    unittest.main()
