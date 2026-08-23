# Orbbec Gemini 336 RGB-D setup

The Gemini 336 is driven through Orbbec's ROS 2 wrapper and exposed to this project through the `legwheel_rgbd` package. Keep the vendor driver as a workspace dependency; put LegWheel-specific launch files, topic choices, remaps, and processing nodes in `legwheel_rgbd`.

## Supported driver path

- Driver: `orbbec/OrbbecSDK_ROS2`
- Branch: `v2-main`
- Camera launch file: `gemini_330_series.launch.py`
- LegWheel wrapper launch file: `ros2 launch legwheel_rgbd gemini_336.launch.py`

The Orbbec wrapper currently lists the Gemini 336 in the Gemini 330 series and recommends the `gemini_330_series.launch.py` launch file.

## Install workspace dependencies

From the repository root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
vcs import . < dependencies.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If `vcs` is missing:

```bash
sudo apt update
sudo apt install python3-vcstool python3-rosdep
```

## Install USB access rules

Linux needs Orbbec udev rules so the camera can be opened without running ROS as root:

```bash
cd src/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and reconnect the camera after installing the rules.

## Verify the camera

```bash
source install/setup.bash
ros2 run orbbec_camera list_devices_node
ros2 launch legwheel_rgbd gemini_336.launch.py
```

In another terminal:

```bash
source install/setup.bash
ros2 topic list | grep legwheel_rgbd
ros2 topic hz /legwheel_rgbd/color/image_raw
ros2 topic hz /legwheel_rgbd/depth/image_raw
ros2 topic echo --once /legwheel_rgbd/depth/camera_info
```

Optional point cloud:

```bash
ros2 launch legwheel_rgbd gemini_336.launch.py enable_point_cloud:=true
```

## Expected topics

With the default `camera_name:=legwheel_rgbd`, Orbbec topics are namespaced under `/legwheel_rgbd`, including:

- `/legwheel_rgbd/color/image_raw`
- `/legwheel_rgbd/color/camera_info`
- `/legwheel_rgbd/depth/image_raw`
- `/legwheel_rgbd/depth/camera_info`
- `/legwheel_rgbd/depth/points` when point cloud output is enabled

## Notes for implementation

- Depend on ROS messages and topics from `orbbec_camera`; do not call the Orbbec SDK directly from LegWheel code unless a ROS topic/service cannot provide the data.
- Keep reusable camera parameters in `src/legwheel_rgbd/config/gemini_336.yaml`.
- Use `serial_number` or `usb_port` launch arguments once multiple cameras are connected.
- Start with point clouds disabled during bring-up to reduce USB and CPU load.
- The Gemini 336 should be on a USB 3 port. If frames drop at high resolution, lower `color_width`, `color_height`, `depth_width`, `depth_height`, or FPS in the config file.
