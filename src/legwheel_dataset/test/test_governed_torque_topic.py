"""New torque targets must stay optional for historical MCAP recordings."""

from types import SimpleNamespace

import pytest

from legwheel_dataset import bag_reader


@pytest.mark.parametrize("with_governed_output", [False, True])
def test_indexing_keeps_requested_and_governed_targets_separate(monkeypatch, with_governed_output):
    topics = set(bag_reader.INDEX_TOPICS)
    rows = [(bag_reader.REQUESTED_TORQUE_TOPIC, SimpleNamespace(data=-2.0), 10)]
    if with_governed_output:
        topics.add(bag_reader.COMMANDED_TORQUE_TOPIC)
        rows.append((bag_reader.COMMANDED_TORQUE_TOPIC, SimpleNamespace(data=-0.7), 11))
    reader = SimpleNamespace(
        get_all_topics_and_types=lambda: [
            SimpleNamespace(name=topic, type="unused") for topic in topics
        ],
        has_next=lambda: bool(rows),
        read_next=lambda: rows.pop(0),
    )
    monkeypatch.setattr(bag_reader, "_open_reader", lambda _: reader)
    monkeypatch.setattr(bag_reader, "_deserialize", lambda message, _: message)
    index = bag_reader.index_bag("unused")
    assert index.streams["torque_command"].arrays()[1][:, 0].tolist() == [-2.0]
    if with_governed_output:
        assert index.streams["governed_torque_command"].arrays()[1][:, 0].tolist() == [-0.7]
    else:
        assert "governed_torque_command" not in index.streams


def test_analysis_reports_the_cap_and_only_finite_commands_inside_the_drive_phase():
    from legwheel_dataset.processor import _describe_wheel_control

    stream = bag_reader.TelemetrySeries(("wheel.commanded_torque",))
    for timestamp, command in [(0, -8.0), (10, -0.5), (20, -1.0), (30, float("nan")), (40, -8.0)]:
        stream.append(timestamp, [command])
    report = _describe_wheel_control(
        SimpleNamespace(streams={"governed_torque_command": stream}),
        {"control_mode": "raw_torque_velocity_capped", "max_wheel_speed_rad_s": 3.0,
         "velocity_governor_kp": 0.5, "velocity_governor_ki": 0.1},
        10, 30,
    )
    assert report["control_mode"] == "raw_torque_velocity_capped"
    assert report["max_wheel_speed_rad_s"] == 3.0
    assert report["interval_source"] == "requested_torque_plateau"
    assert report["governed_command"] == {
        "sample_count": 2, "minimum_nm": -1.0, "maximum_nm": -0.5,
        "mean_absolute_nm": 0.75,
    }
    legacy = _describe_wheel_control(SimpleNamespace(streams={}), {}, 10, 30)
    assert legacy["control_mode"] == "raw_torque"
    assert not legacy["governed_command_available"]
    assert legacy["velocity_governor_kp"] is None
