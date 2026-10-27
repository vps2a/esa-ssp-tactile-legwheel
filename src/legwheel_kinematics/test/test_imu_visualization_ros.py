"""Exercise real ROS message consumers without starting DDS or bag playback."""

from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("rclpy")
from sensor_msgs.msg import JointState

from legwheel_kinematics.config import KinematicsConfig
from legwheel_kinematics.kinematic_joint_state_node import KinematicJointStateNode
from legwheel_kinematics.kinematics_visualization_node import (
    KinematicsVisualizationNode,
)
from legwheel_kinematics.pose_history import PoseHistory, stamp_ns
from legwheel_kinematics.track import generate_track_points
from legwheel_kinematics.track_overlay_node import TrackOverlayNode
from legwheel_kinematics.transforms import KINEMATIC_FRAME_IDS, frame_coordinates


def state(timestamp_ns, yaw=None, pitch=None):
    message = JointState()
    message.header.stamp.sec = timestamp_ns // 1_000_000_000
    message.header.stamp.nanosec = timestamp_ns % 1_000_000_000
    if yaw is not None:
        message.name = ["theta_p_joint", "theta_y_joint"]
        message.position = [pitch, yaw]
    return message


def geometry():
    return KinematicsConfig(
        d1=0.353,
        a_b=0.83,
        h_b=0.0661,
        sensor_front_offset=0.043,
        camera_height=0.029,
        camera_beam_offset=0.054,
        camera_angle_deg=40.0,
        Beam_radius=0.83,
        Wheel_width=0.1,
        wheel_radius_mm=90.0,
        hip_link_length_m=0.2,
        calf_link_length_m=0.2,
        theta_y_kinematic_sign=-1,
    )


def test_shared_publisher_preserves_stamps_and_rejection_barriers_without_revisions():
    publisher = object.__new__(KinematicJointStateNode)
    published = []
    trajectory = SimpleNamespace(
        times_ns=np.array([1_000_000_000, 1_010_000_000, 1_500_000_000]),
        theta_p_rad=np.array([0.12, 0.15, 0.20]),
        valid=np.array([True, False, True]),
    )
    publisher._buffer = SimpleNamespace(
        latest_time_ns=2_000_000_000,
        estimate=lambda: trajectory,
        yaw_at=lambda _: -0.7,
    )
    publisher._publisher = SimpleNamespace(publish=published.append)
    publisher._smoothing_delay_s = 0.6
    publisher._last_estimated_time_ns = None
    publisher._last_published_time_ns = None
    publisher._publish_estimate()
    assert [stamp_ns(item.header.stamp) for item in published] == [
        1_000_000_000,
        1_010_000_000,
    ]
    assert published[0].name == ["theta_y_joint", "theta_p_joint"]
    assert list(published[0].position) == pytest.approx([-0.7, 0.12])
    # Unreliable pitch is an empty update, never a knee pose or a NaN TF.
    assert not published[1].name and not published[1].position
    publisher._publish_estimate()
    assert len(published) == 2
    publisher._buffer.latest_time_ns += 200_000_000
    publisher._publish_estimate()
    assert len(published) == 3
    assert stamp_ns(published[-1].header.stamp) == 1_500_000_000


def test_shared_publisher_recovers_from_invalid_startup_encoder_data():
    publisher = object.__new__(KinematicJointStateNode)
    warnings = []

    def invalid_startup():
        raise ValueError("No finite central encoder angle has been recorded.")

    publisher._buffer = SimpleNamespace(
        latest_time_ns=2_000_000_000, estimate=invalid_startup
    )
    publisher._last_estimated_time_ns = None
    publisher.get_logger = lambda: SimpleNamespace(
        warning=lambda text, **_: warnings.append(text)
    )
    publisher._publish_estimate()
    assert len(warnings) == 1
    assert publisher._last_estimated_time_ns is None


def test_shared_publisher_clears_calibration_and_output_cursor_after_seek():
    publisher = object.__new__(KinematicJointStateNode)
    resets = []
    publisher._buffer = SimpleNamespace(clear=lambda: resets.append(True))
    publisher._using_sim_time = True
    publisher._history_window_ns = 10_000_000_000
    publisher._last_clock_ns = 20_000_000_000
    publisher._last_estimated_time_ns = 19_000_000_000
    publisher._last_published_time_ns = 18_000_000_000
    publisher._clock_callback(SimpleNamespace(clock=state(1_000_000_000).header.stamp))
    assert resets == [True]
    assert publisher._last_estimated_time_ns is None
    assert publisher._last_published_time_ns is None
    assert not publisher._belongs_to_current_playback(20_000_000_000)
    assert publisher._belongs_to_current_playback(1_020_000_000)


def test_3d_geometry_and_both_dynamic_pivot_transforms_use_shared_imu_pose():
    viewer = object.__new__(KinematicsVisualizationNode)
    viewer._config = geometry()
    viewer._track_points = generate_track_points(viewer._config)
    viewer._publish_tf = True
    viewer._using_sim_time = True
    viewer._last_clock_ns = None
    transforms, markers, static_transforms = [], [], []
    viewer._transform_broadcaster = SimpleNamespace(sendTransform=transforms.extend)
    viewer._static_transform_broadcaster = SimpleNamespace(
        sendTransform=static_transforms.extend
    )
    viewer._marker_publisher = SimpleNamespace(publish=markers.append)
    message = state(1_005_000_000, yaw=-0.7, pitch=0.15)
    viewer.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: message.header.stamp)
    )
    viewer._publish_static_transforms()
    assert len(static_transforms) == 3
    assert {item.child_frame_id for item in static_transforms} == set(
        KINEMATIC_FRAME_IDS[3:]
    )
    viewer._kinematic_state_callback(message)
    assert [item.child_frame_id for item in transforms] == list(
        KINEMATIC_FRAME_IDS[1:3]
    )
    assert all(stamp_ns(item.header.stamp) == 1_005_000_000 for item in transforms)
    expected_origins = np.array(
        [matrix[:3, 3] for matrix in frame_coordinates(viewer._config, 0.15, -0.7)]
    )
    origin_marker = next(
        item for item in markers[-1].markers if item.ns == "frame_origins"
    )
    np.testing.assert_allclose(
        [[point.x, point.y, point.z] for point in origin_marker.points],
        expected_origins,
    )
    # An empty derived state is an impact barrier, never a zero or knee pose.
    viewer._kinematic_state_callback(state(1_015_000_000))
    assert len(markers) == 1
    viewer._publish_tf = False
    viewer._kinematic_state_callback(message)
    assert len(transforms) == 2
    assert len(markers) == 2
    viewer._clock_callback(SimpleNamespace(clock=state(0).header.stamp))
    viewer._kinematic_state_callback(message)
    assert len(markers) == 2


def test_overlay_waits_for_image_time_brackets_and_respects_short_impact_barriers():
    overlay = object.__new__(TrackOverlayNode)
    overlay._pose_history = PoseHistory()
    overlay._pending_images = deque(maxlen=3)
    overlay._last_clock_ns = None
    overlay._using_sim_time = True
    published = []
    overlay._publish_marker = lambda image, pitch: published.append(
        (stamp_ns(image.header.stamp), pitch)
    )
    image = SimpleNamespace(header=state(1_005_000_000).header)
    overlay._kinematic_state_callback(state(1_000_000_000, 0.0, 0.0))
    overlay._queue_image(image, object(), object(), 0)
    assert published == []
    overlay._kinematic_state_callback(state(1_010_000_000, 0.0, 0.2))
    assert len(published) == 1
    assert published[0][0] == 1_005_000_000
    assert published[0][1] == pytest.approx(0.1)
    image = SimpleNamespace(header=state(1_015_000_000).header)
    overlay._queue_image(image, object(), object(), 0)
    overlay._kinematic_state_callback(state(1_020_000_000))
    overlay._kinematic_state_callback(state(1_030_000_000, 0.0, 0.3))
    assert len(published) == 1
    overlay._clock_callback(SimpleNamespace(clock=state(2_000_000_000).header.stamp))
    overlay._clock_callback(SimpleNamespace(clock=state(1_000_000_000).header.stamp))
    assert overlay._pose_history.newest_stamp_ns is None
    assert not overlay._pending_images
    assert overlay._rgb_calibration is None
    assert overlay._depth_calibration is None
    overlay._kinematic_state_callback(state(2_010_000_000, 0.0, 0.5))
    assert overlay._pose_history.newest_stamp_ns is None
