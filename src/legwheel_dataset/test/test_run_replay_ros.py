"""Validate real ROS message contracts when running in a sourced ROS workspace."""

from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("rclpy")
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

from legwheel_dataset.run_replay import POINT_DTYPE
from legwheel_dataset.run_replay_node import BASE_FRAME, RunReplayNode, cloud_message
from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.projection import CameraCalibration
from legwheel_kinematics.track import generate_track_points
from legwheel_kinematics.transforms import frame_coordinates


def test_cloud_round_trips_with_exact_xyz_rgb_layout():
    packed = np.array([(1.0, 2.0, 3.0, 0x123456)], dtype=POINT_DTYPE)
    message = cloud_message(packed, 12_345_678_901)
    restored = deserialize_message(serialize_message(message), PointCloud2)
    assert restored.header.frame_id == BASE_FRAME
    assert restored.header.stamp.sec == 12
    assert restored.header.stamp.nanosec == 345_678_901
    assert restored.point_step == 16
    assert restored.row_step == 16
    assert [
        (field.name, field.offset, field.datatype) for field in restored.fields
    ] == [
        ("x", 0, PointField.FLOAT32),
        ("y", 4, PointField.FLOAT32),
        ("z", 8, PointField.FLOAT32),
        ("rgb", 12, PointField.UINT32),
    ]
    assert bytes(restored.data) == packed.tobytes()
    empty = cloud_message(np.array([], dtype=POINT_DTYPE), 0)
    assert deserialize_message(serialize_message(empty), PointCloud2).width == 0


def test_scene_markers_and_transforms_use_only_the_replay_frame_tree():
    geometry = KinematicsConfig(
        d1=0.353,
        a_b=0.83,
        h_b=0.0661,
        sensor_front_offset=0.043,
        camera_height=0.029,
        camera_beam_offset=0.054,
        camera_angle_deg=40.0,
        Beam_radius=0.83,
        Wheel_width=0.1,
        wheel_radius_mm=90,
        hip_link_length_m=0.2,
        calf_link_length_m=0.2,
        theta_y_kinematic_sign=-1,
    )
    calibration = CameraCalibration(
        640,
        480,
        np.array([[460, 0, 320], [0, 460, 240], [0, 0, 1]]),
        np.zeros(5),
        "plumb_bob",
    )
    transforms = []
    # Exercise message construction without starting DDS or a bag player.
    viewer = object.__new__(RunReplayNode)
    viewer.run = SimpleNamespace(
        geometry=geometry,
        frames_at=lambda _: frame_coordinates(geometry, 0.1, theta_y=-0.5),
        angles_at=lambda _: (-0.5, 0.1),
        rgb_calibrations=SimpleNamespace(at=lambda _: calibration),
    )
    viewer.clock_ns = 1_234_000_000
    viewer.alpha_off_rad = 0.35
    viewer.alpha_sp_rad = 0.10
    viewer.track = generate_track_points(geometry)
    viewer.transforms = SimpleNamespace(sendTransform=transforms.extend)
    scene = deserialize_message(serialize_message(viewer.scene()), MarkerArray)
    assert scene.markers[0].action == Marker.DELETEALL
    assert all(marker.header.frame_id == BASE_FRAME for marker in scene.markers)
    assert {"inner_track", "outer_track", "preview_patch", "camera_frustum"} <= {
        marker.ns for marker in scene.markers
    }
    assert len(transforms) == 5
    assert all(
        transform.header.frame_id.startswith("replay_") for transform in transforms
    )
    assert all(
        transform.child_frame_id.startswith("replay_") for transform in transforms
    )
