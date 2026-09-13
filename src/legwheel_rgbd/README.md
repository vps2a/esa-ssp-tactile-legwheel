# legwheel_rgbd: Orbbec Gemini 336 setup

This package launches an Orbbec Gemini 336 RGB-D camera and republishes its
data on the LegWheel camera interface.

It has two layers:

1. `orbbec_camera` from Orbbec's `OrbbecSDK_ROS2` repository opens the USB
   camera and publishes the vendor ROS 2 topics.
2. `legwheel_rgbd` republishes RGB-D data and IMU messages on project-stable
   topics at rates configured for LegWheel.

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
| `config/camera_config.yaml` | LegWheel topic names and outgoing RGB-D/IMU publication rates. |

Before the first run, review `config/gemini_336.yaml`. The shipped settings:

- enable RGB, depth, accelerometer, and gyroscope streams;
- align depth to the colour camera;
- use the camera's default RGB and depth resolution/FPS (`0` values);
- enable frame synchronization and host timestamps; and
- keep point cloud output disabled to reduce USB and CPU load during bring-up.

Set concrete `color_width`, `color_height`, `color_fps`, `depth_width`,
`depth_height`, and `depth_fps` values if the deployment requires a fixed
camera mode. Lower the resolution or FPS first if USB bandwidth is insufficient.

Set the project-facing publication frequencies in `config/camera_config.yaml`:

```yaml
rgbd_publish_rate_hz: 30.0
imu_publish_rate_hz: 100.0
```

The bridge republishes the newest available source message at each configured
tick. It cannot increase the native camera rate, so select values at or below
the RGB-D and IMU rates configured in the Orbbec driver.

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
ros2 topic echo --once /camera/rgbd/depth/camera_info
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
- RGB-D or IMU frequency is lower than expected: check the native driver FPS
  and IMU rates in `config/gemini_336.yaml`; the bridge cannot publish new data
  faster than the device produces it.
- RGB and depth do not line up: keep `depth_registration: true`,
  `align_mode: SW`, and `align_target_stream: COLOR` unless the application
  explicitly needs unaligned depth.

For the current upstream driver documentation and supported camera parameters,
refer to [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2).
