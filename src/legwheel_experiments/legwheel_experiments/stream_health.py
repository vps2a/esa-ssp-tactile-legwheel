"""Timestamp-based health tracking for recorded sensor streams."""

from dataclasses import dataclass


def message_stamp_nanoseconds(message) -> int:
    """Return a stamped ROS message's header time as integer nanoseconds."""
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass
class AdvancingStampMonitor:
    """Track progress using sensor timestamps instead of callback arrival times.

    A cached message can arrive repeatedly and appear fresh to a receipt-time
    watchdog. This monitor updates its freshness time only when the sensor stamp
    strictly increases, making a frozen bridge or camera visible to preflight and
    the runtime safety guard.
    """

    required_advances: int = 3
    last_stamp_ns: int | None = None
    last_advanced_at_s: float | None = None
    advances: int = 0

    def __post_init__(self) -> None:
        if self.required_advances < 1:
            raise ValueError("required_advances must be at least 1")

    def observe(self, stamp_ns: int, received_at_s: float) -> bool:
        """Record a baseline or forward step and report valid stream progress."""
        if stamp_ns <= 0:
            return False

        if self.last_stamp_ns is None:
            self.last_stamp_ns = stamp_ns
            self.last_advanced_at_s = received_at_s
            return True

        if stamp_ns <= self.last_stamp_ns:
            return False

        self.last_stamp_ns = stamp_ns
        self.last_advanced_at_s = received_at_s
        self.advances += 1
        return True

    @property
    def ready(self) -> bool:
        """Return true after several distinct forward timestamp steps."""
        return self.advances >= self.required_advances

    def age_s(self, now_s: float) -> float:
        """Return time since the last advancing stamp, or infinity if absent."""
        if self.last_advanced_at_s is None:
            return float("inf")
        return max(0.0, now_s - self.last_advanced_at_s)
