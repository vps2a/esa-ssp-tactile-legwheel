"""Optional read-only integration test against a real MCAP, without ROS."""

import os
from pathlib import Path

import pytest

from legwheel_dataset.run_replay import CloudReplay, RecordedRun, ReplaySettings


@pytest.mark.skipif(
    not os.environ.get("LEGWHEEL_SAMPLE_RUN"),
    reason="Set LEGWHEEL_SAMPLE_RUN to a recorded run directory.",
)
def test_real_recording_accumulates_and_rebuilds_without_ros():
    mcap = pytest.importorskip("mcap.reader")
    decoder = pytest.importorskip("mcap_ros2.decoder")

    class McapSource:
        """Test adapter; production reads MCAP through the ROS 2 bag API."""

        def __init__(self, path):
            self.stream = path.open("rb")
            self.reader = mcap.make_reader(
                self.stream, decoder_factories=[decoder.DecoderFactory()]
            )
            summary = self.reader.get_summary()
            self.types = {
                channel.topic: summary.schemas[channel.schema_id].name
                for channel in summary.channels.values()
            }

        def messages(self, topics):
            for _, channel, message, decoded in self.reader.iter_decoded_messages(
                topics=list(topics)
            ):
                yield channel.topic, decoded, message.log_time

        def image(self, record):
            return next(
                self.reader.iter_decoded_messages(
                    topics=[record.topic],
                    starting_at=record.log_time_ns,
                    ending_before=record.log_time_ns + 1,
                )
            )[3]

    directory = Path(os.environ["LEGWHEEL_SAMPLE_RUN"])
    paths = list((directory / "rosbag").glob("*.mcap"))
    if not paths:
        pytest.skip("The optional sample recording is not available on this machine.")
    assert len(paths) == 1, "This optional adapter expects a single-file sample bag."
    source = McapSource(paths[0])
    try:
        run = RecordedRun.load(directory, ReplaySettings(), source=source)
        replay = CloudReplay(run)
        end_ns = run.frames[-1].depth.log_time_ns
        while replay.next_index < len(run.frames):
            replay.advance(end_ns)
        # Missing or uncertain IMU intervals are intentionally excluded. A real
        # recording can contain those intervals and still yield useful clouds.
        assert len(replay.failures) < len(run.frames)
        assert len(replay.cloud.points) > 0
        assert not replay.cloud.limit_reached
        complete = replay.cloud.packed().tobytes()
        complete_failures = dict(replay.failures)
        assert (replay.cloud.packed()["rgb"] != 0xA0A0A0).any()
        replay.advance(run.frames[0].depth.log_time_ns - 1)
        assert not replay.cloud.points
        while replay.next_index < len(run.frames):
            replay.advance(end_ns)
        assert replay.cloud.packed().tobytes() == complete
        assert replay.failures == complete_failures
    finally:
        source.stream.close()
