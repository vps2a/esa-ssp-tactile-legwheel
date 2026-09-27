# ESA SSP Tactile LegWheel

ROS 2 Jazzy workspace for operating and recording experiments with the ESA SSP
LegWheel rig. It integrates MAB Robotics MD80 motor controllers, a serial
rotation encoder, an Orbbec Gemini 336 RGB-D camera, and a safety-gated
experiment runner that records MCAP rosbags.

> **Safety notice:** this repository can energise motors and command wheel
> torque. Start with the mechanism secured, the wheel clear of people and
> obstructions, conservative limits, and a tested hardware emergency-stop
> procedure. Software limits are not a substitute for MD80 firmware limits or
> physical safety measures.

## What is in the workspace

| Package | Purpose | Main command |
| --- | --- | --- |
| `legwheel_can` | Connects to MD80s through a CANdle USB-to-CAN adapter; provides passive zeroing, safety-gated leg control, wheel torque control, and terminal controllers. | `ros2 launch legwheel_can md80_legwheel_control.launch.py` |
| `legwheel_encoder` | Reads the central rotation encoder from a serial port and publishes ticks, angle, and velocity. | `ros2 launch legwheel_encoder rotation_encoder.launch.py` |
| `legwheel_rgbd` | Launches and configures the Orbbec driver. Experiments consume its raw RGB, depth, calibration, and IMU topics directly without an image relay. | `ros2 launch legwheel_rgbd gemini_336.launch.py` |
| `legwheel_experiments` | Validates an experiment, performs telemetry preflight, runs the leg/wheel sequence, and records an MCAP rosbag. | `ros2 run legwheel_experiments run_experiment --experiment-config <file>` |

The workspace includes the MAB CANdle-SDK as a Git submodule. Orbbec's ROS 2
driver is downloaded through `dependencies.repos`.

## System requirements

This project is intended for **Ubuntu with ROS 2 Jazzy**. You need:

- a supported Linux computer with USB access;
- ROS 2 Jazzy Desktop or Base installed at `/opt/ros/jazzy`;
- a MAB CANdle USB adapter and correctly configured MD80 controllers;
- the central rotation encoder connected by USB serial;
- an Orbbec Gemini 336 and a USB 3 cable/port when camera data is required;
- sufficient disk space for MCAP recordings (raw RGB-D data is large).

Install build tools and runtime dependencies once:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git libusb-1.0-0-dev \
  python3-vcstool python3-rosdep python3-serial python3-yaml \
  ros-jazzy-rosbag2-storage-mcap
```

Initialize rosdep once per computer. If it says it has already been initialized,
that is fine.

```bash
sudo rosdep init
rosdep update
```

## Installation

Clone the repository and enter its root:

```bash
git clone git@github.com:vps2a/esa-ssp-tactile-legwheel.git
cd esa-ssp-tactile-legwheel
```

Fetch the CANdle-SDK submodule and Orbbec ROS driver, install resolved ROS
dependencies, then build the workspace:

```bash
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
vcs import . < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Run the following in every new terminal before using the workspace:

```bash
cd /path/to/esa-ssp-tactile-legwheel
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Confirm that the workspace-built Orbbec driver is active, rather than a
system-installed copy:

```bash
ros2 pkg prefix orbbec_camera
```

The printed path should end in this workspace's `install/orbbec_camera`.

### USB permissions

Install Orbbec's udev rule once, then unplug and reconnect the camera:

```bash
cd src/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
```

Do not run ROS nodes as root. Ensure the CANdle and serial-encoder devices are
also accessible to the logged-in user. If the encoder appears as `/dev/ttyUSB0`,
the user commonly needs membership of the `dialout` group:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership.

## Configure hardware before enabling it

Review these files before the first real run:

| File | Configure |
| --- | --- |
| `src/legwheel_can/config/motor_ids.yaml` | MD80 CAN IDs for hip, knee, and wheel. |
| `src/legwheel_can/config/motor_limits.yaml` | Software leg position/velocity/torque limits and wheel torque limit. |
| `src/legwheel_can/config/motor_config.json` | Initial leg gains/zero values, wheel torque ramp time, and default wheel torque. |
| `src/legwheel_encoder/config/rotation_encoder.yaml` | Encoder serial port, baud rate, tick calibration, and direction. |
| `src/legwheel_rgbd/config/gemini_336.yaml` | Orbbec stream selection, resolution/FPS, alignment, IMU, and filters. |
| `src/legwheel_experiments/legwheel_experiments/camera_topics.py` | Raw camera topics consumed by preflight, the runtime watchdog, and rosbag. |

The MD80 node requires `limits_provided: true` and valid software limits before
it will enable control. Configure conservative **firmware-level** MD80 current,
torque, velocity, temperature, and watchdog limits separately with MAB's
`candletool`; the ROS YAML limits do not replace them.

After changing installed-package configuration, either rebuild and source the
workspace, or pass the modified YAML file explicitly as a launch argument.

### Rotation-encoder calibration note

The node parameter is named `ticks_per_revolution`, while the shipped YAML uses
`ticks_per_rig_revolution`. Consequently, the shipped `83.3` value is currently
ignored and the node uses its code default of `62.5` ticks/revolution. Rename the
YAML key to `ticks_per_revolution` and set it to the measured calibration before
relying on angle or distance results.

## Bring-up and verification

Bring hardware up one subsystem at a time. The following commands assume the
environment has been sourced as shown above.

### 1. MD80s: passive observation and zeroing

First check encoder feedback without allowing motor power:

```bash
ros2 launch legwheel_can md80_zero.launch.py
```

In another terminal:

```bash
ros2 topic echo /legwheel/joint_states
```

Move the relevant joint manually to its physical zero and call the matching
service. The zeroing node does not enable motors or command motion.

```bash
ros2 service call /legwheel/zero_hip_motor std_srvs/srv/Trigger {}
ros2 service call /legwheel/zero_knee_motor std_srvs/srv/Trigger {}
```

Stop the zeroing node before starting the control node; both publish
`/legwheel/joint_states`.

### 2. MD80 runtime monitoring and control

Start the runtime node in monitoring-only mode. This publishes state but keeps
the motor-control gate closed:

```bash
ros2 launch legwheel_can md80_legwheel_control.launch.py
```

Verify state:

```bash
ros2 topic echo /legwheel/joint_states
ros2 topic echo /wheel/wheel_state
```

To permit control later, restart it with the launch gate armed:

```bash
ros2 launch legwheel_can md80_legwheel_control.launch.py enable_control:=true
```

This still does **not** energise motors. A `true` message on
`/legwheel/motor_status` is also required, and the node refuses to enable if a
motor moved too far while disabled.

### 3. Central rotation encoder

Check the serial device name with `ls /dev/ttyUSB* /dev/ttyACM*`, update
`rotation_encoder.yaml` if necessary, then launch:

```bash
ros2 launch legwheel_encoder rotation_encoder.launch.py
ros2 topic echo /rotation_encoder/joint_state
ros2 topic echo /rotation_encoder/ticks
```

### 4. Gemini 336 RGB-D camera

Confirm that the camera is visible, then launch the project wrapper:

```bash
ros2 run orbbec_camera list_devices_node
ros2 launch legwheel_rgbd gemini_336.launch.py
```

Verify all required experiment-camera streams:

```bash
ros2 topic hz /legwheel_rgbd/color/image_raw
ros2 topic hz /legwheel_rgbd/depth/image_raw
ros2 topic hz /legwheel_rgbd/gyro_accel/sample
ros2 topic echo --once /legwheel_rgbd/depth/camera_info
```

These are the raw Orbbec driver topics. No LegWheel node republishes the images,
so their original content, rate, and `header.stamp` values go directly to the
experiment watchdog and recorder. RGB and depth remain independent and are
paired later during dataset generation. Raw `CameraInfo` messages are recorded
at whatever rate the vendor driver publishes them.

Use `serial_number:=<camera-serial>` or `usb_port:=<port>` with the launch
command when more than one camera is connected. Start with point clouds disabled;
they can be enabled with `enable_point_cloud:=true` when needed.

To use a different RGB-D camera, map its five raw ROS topics in
`src/legwheel_experiments/legwheel_experiments/camera_topics.py`. The expected
interfaces are RGB and depth `sensor_msgs/msg/Image`, matching
`sensor_msgs/msg/CameraInfo`, and one combined `sensor_msgs/msg/Imu`. That single
mapping is imported by both the experiment watchdog and rosbag recorder. Replace
the Orbbec launch/configuration or start the alternative driver separately,
and update `CAMERA_DRIVER_CONFIG_PATH` in the same module so runs snapshot the
replacement configuration (or set it to `None`). Then rebuild and verify rates
and advancing header timestamps. If the API
publishes accelerometer and gyroscope separately, add a lightweight IMU adapter;
do not relay the high-bandwidth image topics merely to rename them. See
`src/legwheel_rgbd/README.md` for the complete integration checklist.

## Manual operation

Manual control is useful for a carefully supervised mechanism check. It is not
the experiment workflow.

1. Start the runtime node with `enable_control:=true`.
2. In a second terminal start the leg command publisher:

   ```bash
   ros2 launch legwheel_can legwheel_controller.launch.py
   ```

3. Set knee impedance parameters, then enable only when the robot and area are
   ready:

   ```text
   knee set_zeropos 0.0
   knee set_spring 4.0
   knee set_damp 0.05
   e
   ```

   The hip mirrors knee encoder position; only knee impedance settings are
   operator-tunable. Type `s` at any time to publish a stop request and disable
   connected motors.

4. For wheel torque testing, launch a third terminal:

   ```bash
   ros2 launch legwheel_can wheel_controller.launch.py torque_limit_nm:=2.0
   ```

   Keep this terminal focused. Hold `w` for one direction, `s` for the other,
   and use `+`/`-` to adjust the requested maximum torque in 0.1 Nm steps.
   Releasing the key ramps torque to zero. The C++ hardware node independently
   clamps torque to `wheel_torque_max_nm`.

The current wheel implementation uses `RAW_TORQUE`, not speed control. Wheel
speed depends on torque, load, supply voltage, and MD80 firmware limits.

## Running a recorded experiment

The experiment runner requires all four hardware data sources to be running:

- `/legwheel/joint_states` and `/wheel/wheel_state` from `legwheel_can`;
- `/rotation_encoder/joint_state` from `legwheel_encoder`;
- RGB, depth, and IMU streams from `legwheel_rgbd`.

It also requires the MD80 runtime node to subscribe to
`/legwheel/motor_status` and `/wheel/requested_torque`. Start the control node
with `enable_control:=true`, plus the encoder and camera launches, before
starting the experiment CLI.

Create an experiment YAML file, for example `experiments/demo.yaml`:

```yaml
schema_version: "1.0"
experiment_id: "demo"
datetime_of_creation: "2026-01-01T12:00:00+00:00"

rig_config:
  beam_radius_m: 0.50
  beam_total_length_m: 1.00
leg_config:
  hip_link_length_m: 0.20
  calf_link_length_m: 0.20
electronics_hardware:
  encoder_setup:
    ticks_per_legwheel_revolution: 62.5
  camera:
    camera_config:
      camera_front_angle_deg: 0.0
wheel_config:
  wheel_radius_mm: 50.0
  wheel_width_mm: 20.0
environment:
  environment_description: "Describe the test surface and conditions."
```

Replace every numeric value with your measured, physically safe rig values.
The runner validates the YAML structure and asks interactively for run length,
knee stiffness/zero/damping, wheel torque, and wheel-ramp duration.

Start it from the repository root:

```bash
ros2 run legwheel_experiments run_experiment \
  --experiment-config "$PWD/experiments/demo.yaml"
```

The runner will:

1. create `run_<n>` beside the experiment YAML and snapshot the experiment,
   camera-driver configuration, and raw camera-topic mapping;
2. hold a telemetry preflight until all motor and encoder streams are fresh,
   and RGB, depth, and IMU have each made three forward header-timestamp steps
   after an initial baseline sample;
3. require explicit `ARM` and `START` confirmations;
4. apply the leg settings, allow optional spring-zero adjustment, then start
   MCAP recording;
5. ramp wheel torque, use the central encoder to determine loop progress, ramp
   torque down, and enter safe mode.

While the experiment is in `RUNNING`, a 10 Hz watchdog checks progress of the
camera sensor timestamps rather than callback arrival time. It immediately
commands zero wheel torque and disables the motors if either image timestamp
does not advance for 2 seconds, or the IMU timestamp does not advance for
0.5 seconds. Repeated cached messages therefore cannot keep a run alive. The
aborted run's `metadata.json` retains the watchdog reason for diagnosis.

Use `Ctrl-C` to abort. The runner sends zero wheel torque, disables motors, and
attempts to finalise the rosbag in its cleanup path.

### Camera recording path

Normal runs record the Orbbec source topics directly. The former
`--record-raw-camera-topics` option and `/camera/...` relay topics have been
removed: recording both source and relayed images duplicated bandwidth while
the Python relay reduced the observed image rate substantially.

## Recorded data and Foxglove

Each run stores `run_config.yaml`, configuration snapshots, `metadata.json`, a
state marker, `rosbag_recorder.log`, and `rosbag/` containing an MCAP bag.
Recorded topics include motor state, wheel state, central encoder data,
independent RGB and depth streams, camera info, IMU, motor-enable state,
impedance commands, and requested wheel torque. Raw camera-info topics may
contain repeated calibration messages because rosbag preserves the driver
output without filtering. RGB/depth association is deliberately deferred to
post-processing and should use the preserved message header timestamps. Do not
infer a pair from adjacent MCAP records, rosbag receive times, or matching
message indices.

The camera topics recorded in every run are:

- `/legwheel_rgbd/color/image_raw`
- `/legwheel_rgbd/color/camera_info`
- `/legwheel_rgbd/depth/image_raw`
- `/legwheel_rgbd/depth/camera_info`
- `/legwheel_rgbd/gyro_accel/sample`

In Foxglove Desktop choose **Open local file** and select the `.mcap` file under
`run_<n>/rosbag/`. Useful panels are:

| Panel | Topics or fields |
| --- | --- |
| Image | `/legwheel_rgbd/color/image_raw`, `/legwheel_rgbd/depth/image_raw` |
| Plot | `/legwheel/joint_states.position[0]`, `.position[1]`, `/wheel/wheel_state.velocity[0]`, `/rotation_encoder/joint_state.position[0]` |
| Plot | `/legwheel_rgbd/gyro_accel/sample/angular_velocity/*`, `/legwheel_rgbd/gyro_accel/sample/linear_acceleration/*` |
| Raw Messages | camera-info, joint-state, torque, and status topics |

Inspect the `name` array in a `JointState` Raw Messages panel to map each array
index to its joint name before interpreting a plot.

## Key topics

| Topic | Message type | Producer | Meaning |
| --- | --- | --- | --- |
| `/legwheel/joint_states` | `sensor_msgs/msg/JointState` | CAN node | Hip and knee state. |
| `/wheel/wheel_state` | `sensor_msgs/msg/JointState` | CAN node | Wheel position, velocity, and torque estimate. |
| `/rotation_encoder/joint_state` | `sensor_msgs/msg/JointState` | Encoder node | Central rig rotation angle and velocity. |
| `/rotation_encoder/ticks` | `std_msgs/msg/Int64` | Encoder node | Relative raw encoder ticks. |
| `/legwheel_rgbd/color/image_raw` | `sensor_msgs/msg/Image` | Orbbec driver | Raw RGB image stream. |
| `/legwheel_rgbd/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Orbbec driver | Raw RGB calibration. |
| `/legwheel_rgbd/depth/image_raw` | `sensor_msgs/msg/Image` | Orbbec driver | Raw depth image stream. |
| `/legwheel_rgbd/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Orbbec driver | Raw depth calibration. |
| `/legwheel_rgbd/gyro_accel/sample` | `sensor_msgs/msg/Imu` | Orbbec driver | Raw combined camera IMU stream. |
| `/legwheel/motor_status` | `std_msgs/msg/Bool` | Controller/experiment | `true` permits the motor node to enable; `false` disables it. |
| `/wheel/requested_torque` | `std_msgs/msg/Float64` | Wheel controller/experiment | Requested wheel torque in Nm; it is clamped by the CAN node. |

## Troubleshooting

- **`Package ... not found`:** source `/opt/ros/jazzy/setup.bash`, then this
  workspace's `install/setup.bash`. If still absent, rebuild.
- **`orbbec_camera` missing or wrong version:** rerun `vcs import . <
  dependencies.repos`, build, source the workspace, and check `ros2 pkg prefix
  orbbec_camera`.
- **Camera cannot open:** install its udev rules, reconnect it, and use a USB 3
  cable/port. Low RGB-D rate should be investigated first on raw
  `/legwheel_rgbd/...` topics with Foxglove and rosbag recording stopped.
- **Camera stamps pause or move backward:** keep `enable_sync_host_time: false`
  with the Gemini driver's default global time domain. On a VM, verify the
  camera is attached to the guest at USB 3 speed with `lsusb -t`, and write
  bags to the guest's native filesystem rather than a shared folder.
- **Encoder cannot open its port:** check `dmesg -w` while reconnecting it,
  correct `port` in the YAML, and check `dialout` access.
- **Motors never enable:** verify `enable_control:=true`, valid limits, a
  `true` status message, fresh feedback, and that the shutdown-position
  interlock has not detected movement.
- **Experiment preflight waits forever:** use `ros2 topic hz` on every required
  stream above, then inspect `header.stamp` on RGB, depth, and IMU. The camera
  checks require three forward timestamp steps, not merely repeated callbacks.
  Images must have advanced within 2 seconds; IMU and motor/encoder state have
  tighter 0.5 second freshness limits.
- **A run aborts for a stopped camera stream:** treat this as a data-integrity
  failure. Keep the generated aborted run and inspect the raw Orbbec rates and
  timestamps with rosbag and Foxglove stopped. If they degrade only while
  recording, inspect CPU and disk throughput and ensure the bag is written to a
  native Linux filesystem rather than a virtual-machine shared folder.

Package-specific implementation and hardware details are available in
[`src/legwheel_can/README.md`](src/legwheel_can/README.md),
[`src/legwheel_rgbd/README.md`](src/legwheel_rgbd/README.md), and
[`docs/orbbec_gemini_336_setup.md`](docs/orbbec_gemini_336_setup.md).
