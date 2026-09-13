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
| `legwheel_rgbd` | Launches the Orbbec driver and republishes RGB, depth, camera calibration, and IMU data on project-stable topics. | `ros2 launch legwheel_rgbd gemini_336.launch.py` |
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
| `src/legwheel_rgbd/config/camera_config.yaml` | Project-facing RGB-D/IMU topics and bridge publication rates. |

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
ros2 topic hz /camera/rgbd/rgb/image_raw
ros2 topic hz /camera/rgbd/depth/image_raw
ros2 topic hz /camera/imu
ros2 topic echo --once /camera/rgbd/depth/camera_info
```

Use `serial_number:=<camera-serial>` or `usb_port:=<port>` with the launch
command when more than one camera is connected. Start with point clouds disabled;
they can be enabled with `enable_point_cloud:=true` when needed.

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

1. create `run_<n>` beside the experiment YAML and snapshot the experiment and
   camera configuration;
2. hold a telemetry preflight until all motor, encoder, RGB, depth, and IMU
   streams are fresh;
3. require explicit `ARM` and `START` confirmations;
4. apply the leg settings, allow optional spring-zero adjustment, then start
   MCAP recording;
5. ramp wheel torque, use the central encoder to determine loop progress, ramp
   torque down, and enter safe mode.

Use `Ctrl-C` to abort. The runner sends zero wheel torque, disables motors, and
attempts to finalise the rosbag in its cleanup path.

## Recorded data and Foxglove

Each run stores `run_config.yaml`, configuration snapshots, `metadata.json`, a
state marker, `rosbag_recorder.log`, and `rosbag/` containing an MCAP bag.
Recorded topics include motor state, wheel state, central encoder data, RGB,
depth, camera info, IMU, motor-enable state, impedance commands, and requested
wheel torque.

In Foxglove Desktop choose **Open local file** and select the `.mcap` file under
`run_<n>/rosbag/`. Useful panels are:

| Panel | Topics or fields |
| --- | --- |
| Image | `/camera/rgbd/rgb/image_raw`, `/camera/rgbd/depth/image_raw` |
| Plot | `/legwheel/joint_states.position[0]`, `.position[1]`, `/wheel/wheel_state.velocity[0]`, `/rotation_encoder/joint_state.position[0]` |
| Plot | `/camera/imu/angular_velocity/*`, `/camera/imu/linear_acceleration/*` |
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
| `/camera/rgbd/rgb/image_raw` | `sensor_msgs/msg/Image` | RGB-D bridge | Project RGB image stream. |
| `/camera/rgbd/depth/image_raw` | `sensor_msgs/msg/Image` | RGB-D bridge | Project depth image stream. |
| `/camera/imu` | `sensor_msgs/msg/Imu` | RGB-D bridge | Camera IMU data. |
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
- **Encoder cannot open its port:** check `dmesg -w` while reconnecting it,
  correct `port` in the YAML, and check `dialout` access.
- **Motors never enable:** verify `enable_control:=true`, valid limits, a
  `true` status message, fresh feedback, and that the shutdown-position
  interlock has not detected movement.
- **Experiment preflight waits forever:** use `ros2 topic hz` on every required
  stream above. Camera images must be newer than one second, and motor/encoder
  state must be newer than 0.5 seconds.

Package-specific implementation and hardware details are available in
[`src/legwheel_can/README.md`](src/legwheel_can/README.md),
[`src/legwheel_rgbd/README.md`](src/legwheel_rgbd/README.md), and
[`docs/orbbec_gemini_336_setup.md`](docs/orbbec_gemini_336_setup.md).
