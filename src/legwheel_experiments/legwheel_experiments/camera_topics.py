"""Raw camera topics consumed by the experiment runner and rosbag recorder.

Keep the camera API boundary in this small module. A deployment using a camera
driver other than Orbbec only needs to update these names, provided the topics
retain the standard ROS message types documented below.
"""

# sensor_msgs/msg/Image
CAMERA_RGB_IMAGE_TOPIC = '/legwheel_rgbd/color/image_raw'
CAMERA_DEPTH_IMAGE_TOPIC = '/legwheel_rgbd/depth/image_raw'

# sensor_msgs/msg/CameraInfo
CAMERA_RGB_INFO_TOPIC = '/legwheel_rgbd/color/camera_info'
CAMERA_DEPTH_INFO_TOPIC = '/legwheel_rgbd/depth/camera_info'

# sensor_msgs/msg/Imu
CAMERA_IMU_TOPIC = '/legwheel_rgbd/gyro_accel/sample'

# Repository-relative driver configuration copied into each run directory.
# Set this to the replacement driver's configuration file, or to None when the
# driver has no file-backed configuration that can be snapshotted.
CAMERA_DRIVER_CONFIG_PATH: str | None = (
    'src/legwheel_rgbd/config/gemini_336.yaml'
)

# CameraInfo is recorded directly from the driver. Unlike the removed relay,
# rosbag does not suppress repeated calibration messages; post-processing may
# retain the first message and any later message whose calibration changed.
CAMERA_RECORDING_TOPICS = (
    CAMERA_RGB_IMAGE_TOPIC,
    CAMERA_RGB_INFO_TOPIC,
    CAMERA_DEPTH_IMAGE_TOPIC,
    CAMERA_DEPTH_INFO_TOPIC,
    CAMERA_IMU_TOPIC,
)
