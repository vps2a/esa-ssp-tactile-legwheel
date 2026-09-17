from legwheel_experiments.rosbag_recorder import (
    RAW_CAMERA_TOPICS,
    ROSBAG_TOPICS,
    topics_for_recording,
)


def test_normal_recording_omits_vendor_camera_topics():
    assert topics_for_recording() == ROSBAG_TOPICS
    assert not set(RAW_CAMERA_TOPICS) & set(ROSBAG_TOPICS)


def test_debug_recording_appends_vendor_camera_topics_once():
    topics = topics_for_recording(include_raw_camera_topics=True)

    assert topics == ROSBAG_TOPICS + RAW_CAMERA_TOPICS
    assert len(topics) == len(set(topics))
