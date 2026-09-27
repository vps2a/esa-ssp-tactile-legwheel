# Orbbec Gemini 336 RGB-D setup

The Gemini 336 is driven by `orbbec/OrbbecSDK_ROS2` and launched through the
`legwheel_rgbd` package.

- Driver source: `src/third_party/OrbbecSDK_ROS2`
- Vendor launch: `gemini_330_series.launch.py`
- LegWheel launch: `ros2 launch legwheel_rgbd gemini_336.launch.py`
- Driver parameters: `src/legwheel_rgbd/config/gemini_336.yaml`
- Experiment topic mapping:
  `src/legwheel_experiments/legwheel_experiments/camera_topics.py`

The experiment runner and rosbag recorder consume the raw Orbbec topics
directly. No relay creates `/camera/...` copies.

## Install

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
vcs import . < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Install the Orbbec udev rule once and reconnect the camera:

```bash
cd src/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
```

## Launch and verify

```bash
ros2 run orbbec_camera list_devices_node
ros2 launch legwheel_rgbd gemini_336.launch.py
```

In another sourced terminal:

```bash
ros2 topic list -t | grep legwheel_rgbd
ros2 topic hz /legwheel_rgbd/color/image_raw
ros2 topic hz /legwheel_rgbd/depth/image_raw
ros2 topic hz /legwheel_rgbd/gyro_accel/sample
ros2 topic echo --once /legwheel_rgbd/depth/camera_info
```

Required topics:

| Topic | Type |
| --- | --- |
| `/legwheel_rgbd/color/image_raw` | `sensor_msgs/msg/Image` |
| `/legwheel_rgbd/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/legwheel_rgbd/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/legwheel_rgbd/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/legwheel_rgbd/gyro_accel/sample` | `sensor_msgs/msg/Imu` |

Point clouds remain disabled by default. Enable them only when needed:

```bash
ros2 launch legwheel_rgbd gemini_336.launch.py enable_point_cloud:=true
```

Use `serial_number:=...` or `usb_port:=...` to select one camera when multiple
devices are attached.

## Timestamp and throughput rules

`enable_frame_sync` is a driver acquisition setting; it does not pair messages
inside the experiment. RGB and depth are recorded independently and paired in
post-processing using their preserved `header.stamp` values.

Keep `enable_sync_host_time: false` with the Gemini driver's default global
time domain. Repeated or regressing raw stamps fail the experiment watchdog.

Use a USB 3 connection. In a virtual machine, `lsusb -t` should report `5000M`
or faster for the camera. Write high-bandwidth bags to a native Linux filesystem
rather than a host-shared directory.

## Using another camera

For another RGB-D driver, identify its five equivalent raw topics with
`ros2 topic list -t`, then edit the constants in
`src/legwheel_experiments/legwheel_experiments/camera_topics.py`. That one
mapping controls both watchdog subscriptions and rosbag recording.

Also update `CAMERA_DRIVER_CONFIG_PATH` in that module so each run snapshots the
new driver's configuration, or set it to `None` if no configuration file exists.

The replacement topics must use `sensor_msgs/msg/Image`,
`sensor_msgs/msg/CameraInfo`, and `sensor_msgs/msg/Imu`. If the driver publishes
separate accelerometer and gyroscope messages, add a lightweight IMU adapter;
do not relay the full RGB and depth images merely to rename them.

Replace the Orbbec launch/configuration or start the alternative driver
separately. Rebuild, source the workspace, and verify topic rates and advancing
header stamps before arming the robot. The detailed integration checklist is in
`src/legwheel_rgbd/README.md`.
