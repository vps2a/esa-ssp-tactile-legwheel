# legwheel_rgbd: Orbbec Gemini 336 setup

This package launches the Orbbec Gemini 336 driver with the settings used by
the LegWheel experiments. The experiment runner and rosbag recorder subscribe
to the driver's raw topics directly.

There is deliberately no Python image relay and no `/camera/...` topic layer.
Relaying full-resolution `sensor_msgs/Image` messages reduced an observed raw
rate of about 15 Hz to 0.5-1 Hz. Direct subscription avoids the extra Python
deserialization, publication, DDS traffic, and memory copies.

The data path is therefore:

```text
Gemini 336 -> orbbec_camera -> /legwheel_rgbd/... raw topics
                                  |-> experiment timestamp preflight
                                  `-> rosbag2 MCAP recorder
```

RGB and depth are not paired online. The original driver messages and their
`header.stamp` values are recorded independently for pairing during dataset
generation.

## Install the camera software

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
vcs import . < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Confirm that ROS resolves the workspace-built driver:

```bash
ros2 pkg prefix orbbec_camera
```

The result should end in this workspace's `install/orbbec_camera`, not
`/opt/ros/jazzy`.

## Install USB access rules

Install Orbbec's udev rule once, then reconnect the camera:

```bash
cd src/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
```

Do not run the camera node as root. The Gemini 336 should use a USB 3 cable and
port. In a virtual machine, `lsusb -t` should report `5000M` or faster for the
camera rather than `480M`.

## Configure and launch the Gemini 336

Driver parameters are kept in `config/gemini_336.yaml`. They select the image
profiles, IMU rates, alignment, filtering, frame synchronization, timestamp
mode, TF, and optional point-cloud output.

The default launch command is:

```bash
ros2 run orbbec_camera list_devices_node
ros2 launch legwheel_rgbd gemini_336.launch.py
```

When upgrading an existing workspace that previously installed the Python
relay, clean only this package's old build/install products before rebuilding
so obsolete console scripts do not remain visible:

```bash
rm -rf build/legwheel_rgbd install/legwheel_rgbd
colcon build --symlink-install \
  --packages-select legwheel_rgbd legwheel_experiments
source install/setup.bash
```

The launch file only includes Orbbec's `gemini_330_series.launch.py`. It does
not start a LegWheel image-processing or forwarding node.

Useful launch options are:

```bash
ros2 launch legwheel_rgbd gemini_336.launch.py \
  serial_number:=CAMERA_SERIAL

ros2 launch legwheel_rgbd gemini_336.launch.py \
  usb_port:=PORT_IDENTIFIER

ros2 launch legwheel_rgbd gemini_336.launch.py \
  enable_point_cloud:=true

ros2 launch legwheel_rgbd gemini_336.launch.py \
  config_file_path:=/absolute/path/to/gemini_336.yaml
```

Use only one of `serial_number` and `usb_port` to select a camera.

## Raw topics used by experiments

With the default `camera_name:=legwheel_rgbd`, the experiment consumes and
records these driver topics:

| Purpose | Topic | Required type |
| --- | --- | --- |
| RGB image | `/legwheel_rgbd/color/image_raw` | `sensor_msgs/msg/Image` |
| RGB calibration | `/legwheel_rgbd/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| Depth image | `/legwheel_rgbd/depth/image_raw` | `sensor_msgs/msg/Image` |
| Depth calibration | `/legwheel_rgbd/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| Combined IMU | `/legwheel_rgbd/gyro_accel/sample` | `sensor_msgs/msg/Imu` |

The single source of truth for these names is
`src/legwheel_experiments/legwheel_experiments/camera_topics.py`. Both the
experiment runner and MCAP recorder import that mapping. The retained runtime
watchdog implementation is currently disabled while an apparent IMU-latency
issue is investigated; timestamp-based preflight checks remain active.

Keep the default `camera_name:=legwheel_rgbd` namespace. If it is changed, the
five constants in `camera_topics.py` must be changed to the resulting topic
names as well.

Verify the driver before running an experiment:

```bash
ros2 topic list -t | grep legwheel_rgbd
ros2 topic hz /legwheel_rgbd/color/image_raw
ros2 topic hz /legwheel_rgbd/depth/image_raw
ros2 topic hz /legwheel_rgbd/gyro_accel/sample
ros2 topic echo --once /legwheel_rgbd/depth/camera_info
```

Inspect sensor timestamps directly when diagnosing stream timing:

```bash
ros2 topic echo /legwheel_rgbd/color/image_raw --field header.stamp
ros2 topic echo /legwheel_rgbd/depth/image_raw --field header.stamp
ros2 topic echo /legwheel_rgbd/gyro_accel/sample --field header.stamp
```

The Gemini 330-series driver uses the global timestamp domain by default. Keep
`enable_sync_host_time: false` with that mode. Repeated or backward timestamps
prevent the experiment timestamp preflight from passing because they make later
sensor correlation ambiguous.

## Connecting a different RGB-D camera API

Do not add another full-image Python forwarding node merely to obtain the
Orbbec topic names. Adapt the experiment at its topic boundary instead:

1. Launch the new vendor driver by itself.
2. Run `ros2 topic list -t` and identify its RGB image, RGB `CameraInfo`, depth
   image, depth `CameraInfo`, and combined IMU topics.
3. Confirm that the image topics use `sensor_msgs/msg/Image`, calibration uses
   `sensor_msgs/msg/CameraInfo`, and IMU uses `sensor_msgs/msg/Imu`.
4. Edit the five topic constants and `CAMERA_DRIVER_CONFIG_PATH` in
   `src/legwheel_experiments/legwheel_experiments/camera_topics.py`. Point the
   latter at the replacement driver's repository configuration, or set it to
   `None` when there is no file-backed configuration to snapshot.
5. Replace this package's Orbbec launch/configuration with the new driver's
   launch, or start that driver separately before starting the experiment.
6. Add the new driver dependency to `dependencies.repos` and/or `package.xml`
   as appropriate, then rebuild and source the workspace.
7. Verify rates, message types, and advancing `header.stamp` values before
   arming the robot.

Changing `camera_topics.py` updates both subscriptions used by preflight and
the list passed to `ros2 bag record`. Each run snapshots this mapping beside the
experiment and, when configured, the camera-driver configuration.

If a camera publishes accelerometer and gyroscope messages separately rather
than a combined `sensor_msgs/msg/Imu`, changing a topic string is insufficient.
Add a small IMU-only adapter that combines the two measurements while
preserving their source timestamps, or change the experiment interface and its
tests explicitly. This is much lighter than relaying RGB and depth images.

If a driver cannot publish standard `sensor_msgs` types, use a narrow adapter
for only the incompatible stream. Keep high-bandwidth images direct whenever
possible.

## Recording behavior

Normal experiment bags contain the five raw camera topics above. There is no
`--record-raw-camera-topics` option because raw topics are now the only camera
source recorded.

The driver may publish `CameraInfo` repeatedly. Rosbag preserves those messages
unchanged; dataset preparation can retain the first calibration and any later
message whose intrinsic or distortion values changed. Images and IMU messages
also remain unchanged, including their original header timestamps.

Do not infer RGB/depth pairs from adjacent MCAP records, matching array indices,
or rosbag receive time. Perform one-to-one association later using the preserved
sensor timestamps and an explicit tolerance.

## Troubleshooting

- If the camera cannot open, install its udev rules, reconnect it, and check the
  USB cable and port.
- If the raw image rate is low, stop Foxglove, RViz, and rosbag; then measure the
  driver topics directly. Check USB speed, CPU load, firmware, and driver logs.
- If rates fall only while recording, write the bag to a native Linux filesystem
  rather than a virtual-machine shared folder.
- Temporarily set `enable_frame_drop_log: true` and `show_fps_enable: true` in
  `gemini_336.yaml` to inspect SDK and ROS publication drops.
- If software alignment is too expensive, test `align_mode: HW` and validate the
  resulting depth geometry before collecting research data.
- If stamps repeat or regress, confirm `enable_sync_host_time: false` and inspect
  the raw driver stream. Do not compensate by replaying cached messages.
