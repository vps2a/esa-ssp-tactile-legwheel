from types import SimpleNamespace

import pytest

from legwheel_experiments.stream_health import (
    AdvancingStampMonitor,
    message_stamp_nanoseconds,
)


def test_monitor_requires_multiple_advancing_timestamps():
    monitor = AdvancingStampMonitor(required_advances=3)

    assert monitor.observe(100, 1.0)
    assert monitor.observe(200, 2.0)
    assert monitor.observe(300, 3.0)
    assert not monitor.ready
    assert monitor.observe(400, 4.0)
    assert monitor.ready


def test_duplicate_and_regressing_stamps_do_not_refresh_stream():
    monitor = AdvancingStampMonitor(required_advances=1)

    assert monitor.observe(100, 1.0)
    assert not monitor.observe(100, 2.0)
    assert not monitor.observe(50, 3.0)
    assert monitor.age_s(4.0) == pytest.approx(3.0)
    assert not monitor.ready


def test_zero_stamp_is_not_valid_progress():
    monitor = AdvancingStampMonitor(required_advances=1)

    assert not monitor.observe(0, 1.0)
    assert monitor.age_s(2.0) == float("inf")


def test_message_stamp_nanoseconds_reads_ros_like_header():
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=7, nanosec=89),
        ),
    )

    assert message_stamp_nanoseconds(message) == 7_000_000_089
