"""Pure helpers for comparing ROS camera metadata and timestamps."""


def stamp_nanoseconds(stamp) -> int:
    """Convert a ROS builtin_interfaces/Time-like object to nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def stamps_strictly_advance(new_stamps, previous_stamps) -> bool:
    """Return whether every positive timestamp advances its matching stream."""
    return all(
        new_stamp > 0
        and (previous_stamp is None or new_stamp > previous_stamp)
        for new_stamp, previous_stamp in zip(new_stamps, previous_stamps)
    )


def camera_info_signature(message) -> tuple:
    """Return calibration fields while intentionally excluding the header stamp.

    Camera drivers commonly publish CameraInfo beside every image, changing only
    its timestamp. Excluding the header lets the bridge suppress those redundant
    messages while still detecting a real calibration, resolution, or ROI change.
    """
    roi = message.roi
    return (
        str(message.header.frame_id),
        int(message.height),
        int(message.width),
        str(message.distortion_model),
        tuple(message.d),
        tuple(message.k),
        tuple(message.r),
        tuple(message.p),
        int(message.binning_x),
        int(message.binning_y),
        int(roi.x_offset),
        int(roi.y_offset),
        int(roi.height),
        int(roi.width),
        bool(roi.do_rectify),
    )
