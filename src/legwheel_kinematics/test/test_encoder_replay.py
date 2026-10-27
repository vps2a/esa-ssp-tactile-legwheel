"""Recorded timestamps and recording zero must survive late starts and seeks."""

import math
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from legwheel_encoder.config import EncoderCalibration
from legwheel_kinematics.encoder_replay import RecordedEncoderTicks
from legwheel_kinematics.imu_pitch_buffer import ImuPitchBuffer
from legwheel_kinematics.config import KinematicsConfig


def geometry():
    return KinematicsConfig(
        d1=0.353,
        a_b=0.83,
        h_b=0.0661,
        sensor_front_offset=0.043,
        camera_height=0.029,
        camera_beam_offset=0.054,
        camera_angle_deg=40,
        Beam_radius=0.83,
        Wheel_width=0.1,
        wheel_radius_mm=90,
        hip_link_length_m=0.2,
        calf_link_length_m=0.2,
        theta_y_kinematic_sign=-1,
    )


def test_window_uses_exact_bag_times_and_keeps_the_complete_recording_origin():
    epoch = 1_790_000_000_000_000_000
    # Input order and callback/clock cadence must not choose a different origin.
    samples = [(epoch + 203, 127), (epoch + 7, 120), (epoch + 101, 125)]
    timeline = RecordedEncoderTicks.from_samples(samples, EncoderCalibration(62.5))
    assert timeline.origin_ticks == 120
    rows = list(timeline.window(epoch + 100, epoch + 210))
    assert [int(row[0]) for row in rows] == [epoch + 101, epoch + 203]
    assert [row[1] for row in rows] == pytest.approx(
        np.array([5, 7]) * 2 * math.pi / 62.5
    )


def test_seek_refill_does_not_rezero_the_streaming_pose_buffer():
    timeline = RecordedEncoderTicks.from_samples(
        [(index * 10_000_000, 120 + index) for index in range(201)],
        EncoderCalibration(62.5),
    )
    buffer = ImuPitchBuffer(geometry(), encoder_origin_motion_rad=0.0)
    for start_ns, end_ns in [
        (1_000_000_000, 2_000_000_000),
        (500_000_000, 1_500_000_000),
    ]:
        buffer.clear()
        for timestamp_ns, angle in timeline.window(start_ns, end_ns):
            buffer.add_encoder(int(timestamp_ns), float(angle))
            buffer.add_wheel(int(timestamp_ns), -1.0)
        assert buffer.yaw_at(start_ns) == pytest.approx(
            -start_ns / 1e7 * 2 * math.pi / 62.5
        )


def test_full_revolutions_are_preserved_instead_of_unwrapped_away():
    timeline = RecordedEncoderTicks.from_samples(
        [(1, 120), (2, 245)], EncoderCalibration(62.5)
    )
    assert timeline.motion_angles_rad[-1] == pytest.approx(4 * math.pi)


@pytest.mark.parametrize("samples", [[], [(1, 20)], [(1, 20), (1, 21)]])
def test_missing_or_duplicate_only_ticks_fail_explicitly(samples):
    with pytest.raises(ValueError, match="samples|timestamps"):
        RecordedEncoderTicks.from_samples(samples, EncoderCalibration(62.5))


def test_bag_adapter_reads_int64_counts_at_original_storage_times(
    tmp_path, monkeypatch
):
    """Verify the I/O contract without requiring DDS or a ROS installation."""
    tick_topic = "/rotation_encoder/ticks"
    records = iter([(tick_topic, 120, 1_000_000_007), (tick_topic, 125, 2_000_000_013)])

    class Reader:
        def open(self, storage, converter):
            assert storage.uri == str(tmp_path)
            assert storage.storage_id == "mcap"

        def get_all_topics_and_types(self):
            return [SimpleNamespace(name=tick_topic, type="std_msgs/msg/Int64")]

        def set_filter(self, topic_filter):
            assert topic_filter.topics == [tick_topic]
            self.remaining = 2

        def has_next(self):
            return self.remaining > 0

        def read_next(self):
            self.remaining -= 1
            return next(records)

    monkeypatch.setitem(
        sys.modules,
        "rosbag2_py",
        SimpleNamespace(
            SequentialReader=Reader,
            StorageOptions=SimpleNamespace,
            ConverterOptions=lambda *_: None,
            StorageFilter=SimpleNamespace,
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "rclpy.serialization",
        SimpleNamespace(
            deserialize_message=lambda count, _: SimpleNamespace(data=count),
        ),
    )
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(Int64=object))
    timeline = RecordedEncoderTicks.from_bag(tmp_path, EncoderCalibration(62.5))
    np.testing.assert_array_equal(timeline.times_ns, [1_000_000_007, 2_000_000_013])
    np.testing.assert_allclose(timeline.motion_angles_rad, [0, 5 * 2 * math.pi / 62.5])
