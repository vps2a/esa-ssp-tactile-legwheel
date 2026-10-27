"""Match image acquisition times to the shared, available IMU pose stream.

This small numerical helper has no ROS dependency. In particular, it never
substitutes the latest pose for a missing pose at an older image timestamp.
The shared publisher retains finite low-confidence poses. An empty JointState
marks only a timestamp with no estimate, so real missing data is not bridged.
"""

from bisect import bisect_left
import math


KINEMATIC_STATE_TOPIC = "/legwheel/kinematic_joint_states"


def joint_state_pose(message):
    """Read named yaw/pitch positions, independently of JointState name order."""
    positions = dict(zip(message.name, message.position))
    try:
        yaw = float(positions["theta_y_joint"])
        pitch = float(positions["theta_p_joint"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "IMU kinematic state must contain theta_y_joint and theta_p_joint positions."
        ) from error
    if not math.isfinite(yaw) or not math.isfinite(pitch):
        raise ValueError("IMU kinematic state contains a nonfinite yaw or pitch.")
    return yaw, pitch


def stamp_ns(stamp):
    """Convert a ROS timestamp without importing any ROS message classes."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class PoseHistory:
    """Retain bounded pose history and interpolate only supported timestamps."""

    def __init__(self, maximum_gap_s=0.05, maximum_samples=2000):
        if not math.isfinite(maximum_gap_s) or maximum_gap_s <= 0:
            raise ValueError("maximum_gap_s must be finite and positive.")
        if maximum_samples < 2:
            raise ValueError("maximum_samples must be at least two.")
        self.maximum_gap_ns = int(round(maximum_gap_s * 1e9))
        self.maximum_samples = maximum_samples
        self._times_ns = []
        self._angles = []

    @property
    def newest_stamp_ns(self):
        """Newest pose time, or None while waiting for the shared publisher."""
        return self._times_ns[-1] if self._times_ns else None

    def clear(self):
        """Discard poses from the previous playback epoch after a clock rewind."""
        self._times_ns.clear()
        self._angles.clear()

    def append(self, timestamp_ns, yaw, pitch):
        """Insert in timestamp order, tolerating duplicate or delayed delivery."""
        if not math.isfinite(yaw) or not math.isfinite(pitch):
            raise ValueError("Pose history requires finite yaw and pitch.")
        self._insert(timestamp_ns, (float(yaw), float(pitch)))

    def append_unavailable(self, timestamp_ns):
        """Keep an unavailable estimate as a barrier rather than bridging it."""
        self._insert(timestamp_ns, None)

    def _insert(self, timestamp_ns, angles):
        index = bisect_left(self._times_ns, timestamp_ns)
        if index < len(self._times_ns) and self._times_ns[index] == timestamp_ns:
            self._angles[index] = angles
        else:
            self._times_ns.insert(index, int(timestamp_ns))
            self._angles.insert(index, angles)
        excess = len(self._times_ns) - self.maximum_samples
        if excess > 0:
            del self._times_ns[:excess]
            del self._angles[:excess]

    def sample(self, timestamp_ns):
        """Interpolate yaw/pitch without extrapolating or crossing a data gap."""
        if (
            not self._times_ns
            or timestamp_ns < self._times_ns[0]
            or timestamp_ns > self._times_ns[-1]
        ):
            raise ValueError(
                "Image timestamp is outside the available IMU pose history."
            )
        right = bisect_left(self._times_ns, timestamp_ns)
        if self._times_ns[right] == timestamp_ns:
            if self._angles[right] is None:
                raise ValueError("IMU pose is unavailable at the image timestamp.")
            return self._angles[right]
        left = right - 1
        if self._angles[left] is None or self._angles[right] is None:
            raise ValueError("Image timestamp crosses an unavailable IMU pose interval.")
        interval_ns = self._times_ns[right] - self._times_ns[left]
        if interval_ns > self.maximum_gap_ns:
            raise ValueError("Image timestamp crosses a gap in available IMU poses.")
        fraction = (timestamp_ns - self._times_ns[left]) / interval_ns
        return tuple(
            earlier + fraction * (later - earlier)
            for earlier, later in zip(self._angles[left], self._angles[right])
        )
