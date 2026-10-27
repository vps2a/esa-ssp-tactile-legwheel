"""Encoder-anchored wheel odometry shared by the visualization paths.

The dataset corrects wheel slip separately between central-encoder changes.
For an interval [a, b], exactly the same calculation can be written as:

    yaw(t) = yaw(a) + (yaw(b) - yaw(a)) * integral(a, t) / integral(a, b)

Here each integral is measured wheel angular velocity integrated over time.
The effective wheel radius / track radius cancels in this ratio. Consequently
neither wheel slip nor a radius calibration can accumulate yaw drift: every
encoder change remains an exact anchor, while wheel motion supplies the timing
between changes. Encoder angles must already have their origin and sign applied.

Unlike dataset selection, visualization keeps the whole recording and both
directions of travel. This module does not import or change dataset processing.
"""

import math

import numpy as np


def _series(times_ns, values, name):
    """Require ordered, unique samples; retain nonfinite values as data gaps."""
    times = np.asarray(times_ns, dtype=np.int64)
    values = np.asarray(values, dtype=float)
    if times.ndim != 1 or values.shape != times.shape:
        raise ValueError(f"{name} timestamps and values must be matching vectors.")
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ValueError(f"{name} needs at least two increasing timestamps.")
    return times, values


class EncoderYawTrajectory:
    """Refine continuous central-encoder angles with measured wheel velocity.

    Repeated encoder messages are coverage information, not additional angular
    anchors: anchoring every unchanged message would recreate a staircase and
    erase the wheel refinement. The first/last samples close recording edges.
    Missing wheel support leaves the measured encoder pose available, without
    refinement. Missing encoder support remains unavailable; neither input is
    extrapolated. This lets full-run views include slow wheel telemetry outside
    the dataset's selected motion interval without rejecting measured poses.
    """

    def __init__(
        self,
        encoder_times_ns,
        encoder_angles_rad,
        wheel_times_ns,
        wheel_velocity_rad_s,
        *,
        maximum_encoder_gap_s=0.5,
        maximum_wheel_gap_s=0.05,
    ):
        for name, value in (
            ("maximum_encoder_gap_s", maximum_encoder_gap_s),
            ("maximum_wheel_gap_s", maximum_wheel_gap_s),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive.")
        self._encoder_times, self._encoder_angles = _series(
            encoder_times_ns, encoder_angles_rad, "Central encoder"
        )
        wheel_times, wheel_velocity = _series(
            wheel_times_ns, wheel_velocity_rad_s, "Wheel odometry"
        )
        self._maximum_encoder_gap_ns = round(maximum_encoder_gap_s * 1e9)

        # Preserve full count-derived revolutions and real direction changes.
        # Nonfinite observations become interval boundaries so recovery after
        # a bad reading does not connect through it to an earlier good anchor.
        finite = np.isfinite(self._encoder_angles)
        changed = np.r_[
            True,
            (np.abs(np.diff(self._encoder_angles)) > 1e-12)
            | ~finite[:-1]
            | ~finite[1:],
        ]
        changed[-1] = True
        self._anchor_times = self._encoder_times[changed]
        self._anchor_angles = self._encoder_angles[changed]

        # Integrate once at every wheel sample and encoder anchor. Subtract an
        # integer epoch before converting to floats to preserve timestamp detail.
        # Adding encoder anchors as knots makes partial boundary trapezoids exact.
        self._knots = np.union1d(wheel_times, self._anchor_times)
        origin_ns = int(self._knots[0])
        self._velocity = np.interp(
            (self._knots - origin_ns).astype(float),
            (wheel_times - origin_ns).astype(float),
            wheel_velocity,
            left=np.nan,
            right=np.nan,
        )
        self._durations_s = np.diff(self._knots).astype(float) * 1e-9
        wheel_intervals = np.clip(
            np.searchsorted(wheel_times, self._knots[:-1], side="right") - 1,
            0,
            len(wheel_times) - 2,
        )
        supported = (
            (self._knots[:-1] >= wheel_times[0])
            & (self._knots[1:] <= wheel_times[-1])
            & (
                np.diff(wheel_times)[wheel_intervals]
                <= round(maximum_wheel_gap_s * 1e9)
            )
            & np.isfinite(self._velocity[:-1])
            & np.isfinite(self._velocity[1:])
        )
        increments = (
            0.5 * (self._velocity[:-1] + self._velocity[1:]) * self._durations_s
        )
        # A separate invalid-interval counter prevents a finite cumulative sum
        # from accidentally making an interval through missing telemetry usable.
        self._integrals = np.r_[0.0, np.cumsum(np.where(supported, increments, 0.0))]
        self._gap_counts = np.r_[0, np.cumsum(~supported)]
        self._anchor_knots = np.searchsorted(self._knots, self._anchor_times)

    def sample(self, timestamp_ns: int) -> float:
        """Return theta_y in radians at an acquisition time, without extrapolation."""
        right = int(np.searchsorted(self._encoder_times, timestamp_ns))
        if right == len(self._encoder_times) or timestamp_ns < self._encoder_times[0]:
            raise ValueError("Encoder yaw is unavailable outside recorded coverage.")
        if self._encoder_times[right] == timestamp_ns:
            if not np.isfinite(self._encoder_angles[right]):
                raise ValueError(
                    "Encoder yaw is unavailable at a nonfinite measurement."
                )
            encoder_angle = float(self._encoder_angles[right])
        elif (
            self._encoder_times[right] - self._encoder_times[right - 1]
            > self._maximum_encoder_gap_ns
            or not np.isfinite(self._encoder_angles[right - 1 : right + 1]).all()
        ):
            raise ValueError("Encoder yaw is unavailable across a data gap.")
        else:
            # Keep the previous visualizer's central-encoder interpolation as
            # the measured pose when wheel refinement cannot be calculated.
            left = right - 1
            fraction = (timestamp_ns - int(self._encoder_times[left])) / int(
                self._encoder_times[right] - self._encoder_times[left]
            )
            encoder_angle = float(
                self._encoder_angles[left]
                + fraction * (self._encoder_angles[right] - self._encoder_angles[left])
            )

        anchor = (
            int(np.searchsorted(self._anchor_times, timestamp_ns, side="right")) - 1
        )
        start_angle = float(self._anchor_angles[anchor])
        if timestamp_ns == self._anchor_times[anchor]:
            return start_angle
        encoder_delta = float(self._anchor_angles[anchor + 1] - start_angle)
        if not math.isfinite(start_angle) or not math.isfinite(encoder_delta):
            return encoder_angle
        if encoder_delta == 0.0:
            # A stationary final interval stays stationary, including wheel
            # spin in place: odometry must never override the encoder anchors.
            return start_angle

        first, last = self._anchor_knots[anchor : anchor + 2]
        if self._gap_counts[last] != self._gap_counts[first]:
            return encoder_angle
        whole_integral = float(self._integrals[last] - self._integrals[first])
        if abs(whole_integral) <= 1e-12:
            # A zero net wheel integral cannot be normalized. The encoder is
            # still a valid measurement; never divide by zero or discard it.
            return encoder_angle

        knot = int(np.searchsorted(self._knots, timestamp_ns, side="right")) - 1
        elapsed_s = (int(timestamp_ns) - int(self._knots[knot])) * 1e-9
        # Velocity is linear between samples, so its partial integral is a
        # trapezoid too. Interpolating cumulative angles linearly would wrongly
        # assume constant velocity and lose accuracy during acceleration.
        velocity_change = self._velocity[knot + 1] - self._velocity[knot]
        velocity_at_time = (
            self._velocity[knot] + velocity_change * elapsed_s / self._durations_s[knot]
        )
        partial_integral = (
            self._integrals[knot]
            - self._integrals[first]
            + 0.5 * (self._velocity[knot] + velocity_at_time) * elapsed_s
        )
        return float(start_angle + encoder_delta * partial_integral / whole_integral)
