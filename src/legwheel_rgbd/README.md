# legwheel_rgbd: Orbbec Gemini 336 setup

This package launches an Orbbec Gemini 336 RGB-D camera and republishes its
data on the LegWheel camera interface.

It has two layers:

1. `orbbec_camera` from Orbbec's `OrbbecSDK_ROS2` repository opens the USB
   camera and publishes the vendor ROS 2 topics.
2. `legwheel_rgbd` publishes a cleaned project interface: it pairs RGB and
   depth by sensor timestamp, forwards each paired frame and IMU sample once,
   and publishes camera calibration only when it changes.

The Orbbec ROS 2 wrapper is an external workspace dependency. It is declared
in the repository-root `dependencies.repos` file, so it is downloaded and
built with the workspace rather than copied into this package.

## Prerequisites

Use a Linux machine with:

- Ubuntu and ROS 2 installed. This workspace is configured for ROS 2 Jazzy.
- A USB 3 port and a USB 3 cable for the Gemini 336.
- Internet access while preparing the workspace.

Install the workspace dependency tools once:

```bash
sudo apt update
sudo apt install python3-vcstool python3-rosdep
```

If `rosdep` has not been initialized on the machine, initialize it once and
then update its package index:

```bash
sudo rosdep init
rosdep update
```

`sudo rosdep init` may report that it has already been initialized. In that
case, continue with `rosdep update`.

## Install the camera software

Run these commands from the root of this repository, not from this package
directory:

```bash
source /opt/ros/jazzy/setup.bash
vcs import . < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`vcs import` checks out Orbbec's ROS 2 wrapper at
`src/third_party/OrbbecSDK_ROS2`. The build installs its `orbbec_camera` ROS 2
package alongside `legwheel_rgbd`. No separate application-level Orbbec SDK
installation is required for this integration.

### Confirm the workspace driver is selected

Always source the workspace after the base ROS installation:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix orbbec_camera
```

The final command must print a path inside this workspace, ending in
`install/orbbec_camera`. A result under `/opt/ros/jazzy` means ROS is using a
system-installed Orbbec driver instead. That binary can be incompatible with
the installed ROS libraries and must not be mixed with this workspace's source
build. Re-run `vcs import`, build the workspace, and source `install/setup.bash`
again before launching the camera.

To refresh the imported source later, run:

```bash
vcs pull src/third_party/OrbbecSDK_ROS2
```

Do this deliberately and rebuild afterward: driver updates can change camera
parameters or supported firmware.

## Allow non-root USB access

Install Orbbec's udev rule once on every Linux machine that will use the
camera:

```bash
cd src/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
```

The installer copies the `99-obsensor-libusb.rules` rule and reloads udev.
Unplug and reconnect the Gemini 336 after it completes. Do not run the camera
node as root.

## Configure the camera

There are two configuration files with separate responsibilities:

| File | Purpose |
| --- | --- |
| `config/gemini_336.yaml` | Orbbec driver settings: enabled streams, alignment, resolution, frame rate, IMU, filters, and TF. |
| `config/camera_config.yaml` | LegWheel topic names and RGB/depth synchronization settings. |

Before the first run, review `config/gemini_336.yaml`. The shipped settings:

- enable RGB, depth, accelerometer, and gyroscope streams;
- align depth to the colour camera;
- request 640 x 480 RGB and depth streams at 30 FPS;
- enable frame synchronization and host timestamps; and
- keep point cloud output disabled to reduce USB and CPU load during bring-up.

Set concrete `color_width`, `color_height`, `color_fps`, `depth_width`,
`depth_height`, and `depth_fps` values if the deployment requires a fixed
camera mode. Lower the resolution or FPS first if USB bandwidth is insufficient.

The observed camera rate can be lower and irregular even when the driver is
configured for 30 FPS. The bridge is therefore event-driven: it never fills a
gap by publishing a cached image or IMU sample again.

Configure RGB/depth pairing in `config/camera_config.yaml`:

```yaml
rgbd_sync_queue_size: 30
rgbd_sync_slop_s: 0.01
```

`rgbd_sync_slop_s` is the maximum allowed difference between RGB and depth
header stamps. The 10 ms default accepts the close pairs produced by the
frame-synchronized driver and rejects an image from the adjacent nominal
30 FPS frame, which is about 33 ms away. The queue accommodates short bursts
and scheduling jitter. Increase the tolerance only after measuring the raw
topic stamps; an overly large value can pair neighbouring, unrelated frames.

The forwarding rules are:

- RGB and depth are published only together after approximate timestamp
  synchronization. A source image is consumed by at most one output pair.
- A zero, repeated, or regressing RGB, depth, or IMU header stamp is dropped.
- IMU samples are forwarded from their source callback, once per advancing
  header stamp.
- Each `CameraInfo` is compared without its header timestamp. The first
  calibration and any changed calibration are published; repeated copies are
  suppressed. The output uses transient-local durability so a recorder or
  subscriber that starts later still receives the latest calibration.

## Run and verify

Connect one Gemini 336, then from the repository root run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run orbbec_camera list_devices_node
ros2 launch legwheel_rgbd gemini_336.launch.py
```

`list_devices_node` should show the attached camera before launch. In a second
terminal, source the same environments and verify the interface:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic hz /camera/rgbd/rgb/image_raw
ros2 topic hz /camera/rgbd/depth/image_raw
ros2 topic hz /camera/imu
ros2 topic echo --once --qos-durability transient_local \
  /camera/rgbd/depth/camera_info
```

RGB and depth should report the same output count and rate because they are
published as pairs. The rate is the rate actually delivered by the camera, not
an artificial 30 Hz. To inspect timestamps directly:

```bash
ros2 topic echo /camera/rgbd/rgb/image_raw --field header.stamp
ros2 topic echo /camera/rgbd/depth/image_raw --field header.stamp
```

Expected LegWheel topics:

- `/camera/rgbd/rgb/image_raw` (`sensor_msgs/Image`)
- `/camera/rgbd/rgb/camera_info` (`sensor_msgs/CameraInfo`)
- `/camera/rgbd/depth/image_raw` (`sensor_msgs/Image`)
- `/camera/rgbd/depth/camera_info` (`sensor_msgs/CameraInfo`)
- `/camera/imu` (`sensor_msgs/Imu`)

The raw Orbbec topics remain available under `/legwheel_rgbd`, including
`/legwheel_rgbd/color/image_raw`, `/legwheel_rgbd/depth/image_raw`, and
`/legwheel_rgbd/gyro_accel/sample`.

The experiment recorder uses the cleaned topics by default. Add
`--record-raw-camera-topics` to `run_experiment` for a diagnostic run that also
captures all five raw image, calibration, and combined-IMU source topics. This
can nearly double camera I/O and bag size, but it lets you compare input and
output header stamps when diagnosing dropped or unpaired frames.

## Launch options

Enable the driver point cloud only when it is needed:

```bash
ros2 launch legwheel_rgbd gemini_336.launch.py enable_point_cloud:=true
```

For a multi-camera machine, select the camera explicitly:

```bash
ros2 launch legwheel_rgbd gemini_336.launch.py serial_number:=CAMERA_SERIAL
```

Alternatively, pass `usb_port:=PORT_IDENTIFIER`. Use one selector at a time.

To use a deployment-specific configuration without modifying the repository
defaults:

```bash
ros2 launch legwheel_rgbd gemini_336.launch.py \
  config_file_path:=/absolute/path/to/gemini_336.yaml \
  camera_config:=/absolute/path/to/camera_config.yaml
```

## Troubleshooting

- `Package 'orbbec_camera' not found`: run `vcs import . < dependencies.repos`,
  rebuild the workspace, and source `install/setup.bash`.
- Camera cannot be opened or no device is listed: confirm the USB 3 cable and
  port, install the udev rule, then reconnect the camera.
- RGB-D or IMU frequency is lower than expected: compare the raw and project
  topics. The bridge cannot publish new data faster than the device produces
  it and deliberately does not repeat cached messages.
- Raw RGB and depth arrive but project RGB-D does not: compare raw header
  stamps. No pair is published when the difference exceeds
  `rgbd_sync_slop_s`; check `enable_frame_sync` before increasing the slop.
- A warning reports a non-advancing timestamp: the source is repeating a
  cached sample, has reset its clock, or is publishing a zero stamp. The bridge
  drops it so it cannot masquerade as new data.
- RGB and depth do not line up: keep `depth_registration: true`,
  `align_mode: SW`, and `align_target_stream: COLOR` unless the application
  explicitly needs unaligned depth.

For the current upstream driver documentation and supported camera parameters,
refer to [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2).
