from types import SimpleNamespace

from legwheel_rgbd.stream_utils import (
    camera_info_signature,
    stamp_nanoseconds,
    stamps_strictly_advance,
)


def make_camera_info(stamp_sec=1, fx=460.0):
    """Build the small CameraInfo-like object required by the pure helper."""
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp_sec, nanosec=25),
            frame_id="camera_color_optical_frame",
        ),
        height=480,
        width=640,
        distortion_model="plumb_bob",
        d=[0.0] * 5,
        k=[fx, 0.0, 320.0, 0.0, fx, 240.0, 0.0, 0.0, 1.0],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=[fx, 0.0, 320.0, 0.0, 0.0, fx, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x=0,
        binning_y=0,
        roi=SimpleNamespace(
            x_offset=0,
            y_offset=0,
            height=0,
            width=0,
            do_rectify=False,
        ),
    )


def test_camera_info_signature_ignores_header_timestamp():
    first = make_camera_info(stamp_sec=1)
    later = make_camera_info(stamp_sec=2)

    assert camera_info_signature(first) == camera_info_signature(later)


def test_camera_info_signature_detects_calibration_change():
    original = make_camera_info(fx=460.0)
    recalibrated = make_camera_info(fx=462.0)

    assert camera_info_signature(original) != camera_info_signature(recalibrated)


def test_camera_info_signature_detects_frame_change():
    original = make_camera_info()
    moved = make_camera_info()
    moved.header.frame_id = "replacement_color_optical_frame"

    assert camera_info_signature(original) != camera_info_signature(moved)


def test_stamp_nanoseconds_combines_seconds_and_nanoseconds():
    stamp = SimpleNamespace(sec=12, nanosec=345)

    assert stamp_nanoseconds(stamp) == 12_000_000_345


def test_stamps_strictly_advance_accepts_a_first_positive_pair():
    assert stamps_strictly_advance((10, 11), (None, None))


def test_stamps_strictly_advance_rejects_zero_repeat_or_regression():
    assert not stamps_strictly_advance((0, 11), (None, None))
    assert not stamps_strictly_advance((10, 12), (10, 11))
    assert not stamps_strictly_advance((9, 12), (10, 11))
