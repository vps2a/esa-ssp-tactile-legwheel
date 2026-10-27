"""Adapt headerless recorded ticks to a timestamped visualization history.

The bag's receipt timestamp is the only recorded time on an Int64 tick message.
Never replace it with callback time or a nearby /clock update. The complete tick
timeline also supplies one fixed recording origin, including after a seek.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from legwheel_encoder.config import TICKS_TOPIC, EncoderCalibration


@dataclass(frozen=True)
class RecordedEncoderTicks:
    times_ns: np.ndarray
    motion_angles_rad: np.ndarray
    origin_ticks: int

    @classmethod
    def from_samples(cls, samples, calibration: EncoderCalibration):
        """Order samples before choosing the first recorded count as zero."""
        rows = sorted(samples, key=lambda row: row[0])
        if len(rows) < 2:
            raise ValueError("The recorded ticks topic needs at least two samples.")
        times = np.asarray([row[0] for row in rows], dtype=np.int64)
        counts = np.asarray([row[1] for row in rows], dtype=np.int64)
        unique = np.r_[True, np.diff(times) > 0]
        times, counts = times[unique], counts[unique]
        if len(times) < 2:
            raise ValueError("The recorded ticks topic needs distinct timestamps.")
        origin_ticks = int(counts[0])
        return cls(times, calibration.motion_angle(counts, origin_ticks), origin_ticks)

    @classmethod
    def from_bag(cls, bag_directory: Path, calibration, topic=TICKS_TOPIC):
        """Read only encoder ticks; images and other telemetry stay on disk."""
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import Int64

        if not Path(bag_directory).is_dir():
            raise ValueError(f"Encoder bag directory does not exist: {bag_directory}")
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""),
            )
            types = {
                entry.name: entry.type for entry in reader.get_all_topics_and_types()
            }
            if types.get(topic) != "std_msgs/msg/Int64":
                raise ValueError(f"Bag must contain {topic} as std_msgs/msg/Int64.")
            reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
            samples = []
            while reader.has_next():
                _, data, bag_time_ns = reader.read_next()
                message = deserialize_message(data, Int64)
                samples.append((int(bag_time_ns), int(message.data)))
            return cls.from_samples(samples, calibration)
        finally:
            # SequentialReader releases the storage handle in its destructor.
            del reader

    def window(self, start_ns: int, end_ns: int):
        """Return recorded samples in an inclusive clock-driven window."""
        first = np.searchsorted(self.times_ns, start_ns, side="left")
        last = np.searchsorted(self.times_ns, end_ns, side="right")
        return zip(self.times_ns[first:last], self.motion_angles_rad[first:last])
