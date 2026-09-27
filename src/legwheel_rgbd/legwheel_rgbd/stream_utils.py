"""Pure helper for comparing ROS camera calibration metadata."""


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
