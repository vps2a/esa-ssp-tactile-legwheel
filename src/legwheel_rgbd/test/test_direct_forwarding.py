from types import SimpleNamespace

from legwheel_rgbd.gemini_336_camera_node import Gemini336CameraNode


class RecordingPublisher:
    """Minimal publisher stand-in that records object identity and call order."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_high_rate_stream_callbacks_forward_every_message_unchanged():
    """Repeated stamps must remain visible rather than being filtered online."""
    node = SimpleNamespace(
        _source_received={"rgb": False, "depth": False, "imu": False},
        rgb_pub=RecordingPublisher(),
        depth_pub=RecordingPublisher(),
        imu_pub=RecordingPublisher(),
    )
    repeated_stamp_message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=345),
        ),
    )

    # Calling each callback twice represents two source deliveries with the same
    # sensor stamp. The bridge must forward both and must not clone or rewrite them.
    Gemini336CameraNode._publish_rgb(node, repeated_stamp_message)
    Gemini336CameraNode._publish_rgb(node, repeated_stamp_message)
    Gemini336CameraNode._publish_depth(node, repeated_stamp_message)
    Gemini336CameraNode._publish_depth(node, repeated_stamp_message)
    Gemini336CameraNode._publish_imu(node, repeated_stamp_message)
    Gemini336CameraNode._publish_imu(node, repeated_stamp_message)

    assert node.rgb_pub.messages == [repeated_stamp_message] * 2
    assert node.depth_pub.messages == [repeated_stamp_message] * 2
    assert node.imu_pub.messages == [repeated_stamp_message] * 2
    assert all(node._source_received.values())
