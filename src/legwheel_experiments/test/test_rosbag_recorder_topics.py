from legwheel_experiments.camera_topics import CAMERA_RECORDING_TOPICS
from legwheel_experiments.rosbag_recorder import ROSBAG_TOPICS


def test_normal_recording_contains_raw_camera_topics_once():
    assert set(CAMERA_RECORDING_TOPICS) <= set(ROSBAG_TOPICS)
    assert len(ROSBAG_TOPICS) == len(set(ROSBAG_TOPICS))


def test_removed_relay_namespace_is_not_recorded():
    assert not any(topic.startswith("/camera/") for topic in ROSBAG_TOPICS)
