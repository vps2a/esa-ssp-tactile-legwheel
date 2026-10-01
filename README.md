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
| `legwheel_description` | Publishes the kinematic frame tree from the experiment geometry and derived `theta_p`. | `ros2 launch legwheel_description description.launch.py experiment_config:=<file>` |
| `legwheel_kinematics` | Implements the verified rig kinematics and publishes track annotations for Foxglove. | `ros2 launch legwheel_kinematics track_overlay.launch.py experiment_config:=<file>` |
| `legwheel_dataset` | Converts recorded MCAP runs into synchronized packets, terrain-slope labels, and telemetry spectra. | `ros2 run legwheel_dataset build_dataset --run-directory <run>` |

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
| `src/legwheel_experiments/legwheel_experiments/camera_topics.py` | Raw camera topics consumed by preflight, the retained runtime-watchdog implementation, and rosbag. |

The MD80 node requires `limits_provided: true` and valid software limits before
it will enable control. Configure conservative **firmware-level** MD80 current,
torque, velocity, temperature, and watchdog limits separately with MAB's
`candletool`; the ROS YAML limits do not replace them.

After changing installed-package configuration, either rebuild and source the
workspace, or pass the modified YAML file explicitly as a launch argument.

### Rotation-encoder calibration note

The node and shipped YAML both use `ticks_per_revolution`. Set the shipped
`83.3` value to the measured calibration before relying on angle, distance, or
camera-to-contact timing results.

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
experiment preflight checks and recorder. RGB and depth remain independent and are
paired later during dataset generation. Raw `CameraInfo` messages are recorded
at whatever rate the vendor driver publishes them.

Use `serial_number:=<camera-serial>` or `usb_port:=<port>` with the launch
command when more than one camera is connected. Start with point clouds disabled;
they can be enabled with `enable_point_cloud:=true` when needed.

To use a different RGB-D camera, map its five raw ROS topics in
`src/legwheel_experiments/legwheel_experiments/camera_topics.py`. The expected
interfaces are RGB and depth `sensor_msgs/msg/Image`, matching
`sensor_msgs/msg/CameraInfo`, and one combined `sensor_msgs/msg/Imu`. That single
mapping is imported by both the experiment runner and rosbag recorder. Replace
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
  beam_radius_m: 0.83
  beam_total_length_m: 1.00
  pivot_height_m: 0.353
leg_config:
  hip_link_length_m: 0.20
  calf_link_length_m: 0.20
electronics_hardware:
  encoder_setup:
    ticks_per_legwheel_revolution: 83.3
  camera:
    camera_config:
      camera_beam_offset_m: -0.043
      camera_height_m: 0.029
      camera_front_angle_deg: 40.0
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

The runtime camera watchdog is temporarily disabled while the reported IMU
latency is investigated. Its implementation remains in
`experiment_runner_node.py`, guarded by `_RUNTIME_CAMERA_WATCHDOG_ENABLED`, but
it does not create its 10 Hz timer or perform the final pre-run camera-health
check while that flag is false. Consequently, a camera stream that stalls after
preflight will not automatically abort an active experiment. Preflight still
requires RGB, depth, and IMU timestamps to advance before arming.

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

## Track projection and offline dataset generation

Two packages keep analysis code out of the motor-control and recording process:

| Package | Purpose |
| --- | --- |
| `legwheel_description` | Optional URDF representation of the notebook frame chain. |
| `legwheel_kinematics` | Loads experiment geometry, derives `theta_p` from `knee_joint`, publishes replay-time TF and 3D track points, and projects the circular track into camera images. |
| `legwheel_dataset` | Reads MCAP directly, isolates synchronized packets, extracts RGB/depth patches, fits terrain planes, and calculates telemetry spectra. |

The experiment YAML must include the geometry shown in the example experiment
configuration above. `h_b` is the measured constant `0.0761 m`, and the camera
offset from the beam centre is the measured constant `0.04 m`; neither is
duplicated in the experiment file.

### Verify the projection in Foxglove

For a recorded run, calculate annotations using simulated ROS time. Start the
two processing launches first in separate terminals, then start bag playback so
the beginning of the recording is not missed:

```bash
ros2 launch legwheel_kinematics track_overlay.launch.py \
  use_sim_time:=true \
  experiment_config:=/path/to/run_1/experiment_config_run_1_snapshot.yaml
```

```bash
ros2 launch legwheel_kinematics kinematics_visualization.launch.py \
  use_sim_time:=true \
  experiment_config:=/path/to/run_1/experiment_config_run_1_snapshot.yaml
```

```bash
ros2 bag play /path/to/run_1/rosbag --clock
```

Connect Foxglove through the normal ROS bridge. In an Image panel select the raw
RGB or depth image and add the corresponding annotation topic:

- `/legwheel/visualization/rgb_track`
- `/legwheel/visualization/depth_track`

The marker has the exact image timestamp. The overlay uses the latest available
named `knee_joint` state and the same numerical projection library as offline
processing.

For the 3D view, add a **3D** panel, set its display frame to
`legwheel_base`, enable `/legwheel/visualization/kinematics_3d`, and enable the
transform frames and labels. The marker topic contains the inner and outer
circular track points, the six notebook frame origins, and a line joining the
kinematic chain. The same replay-only node publishes the fixed transforms on
`/tf_static` and recalculates the `theta_p` transform on `/tf` for every
recorded `knee_joint` sample. The markers and moving transform use that sample's
original header timestamp, so seeking and playback speed changes remain
aligned.

Do not launch `legwheel_description description.launch.py` at the same time as
`kinematics_visualization.launch.py`: both publish the same frame names and
would create competing TF authorities. The direct kinematics visualization
launch is preferred for rosbag post-processing and does not require TF to have
been recorded in the raw bag.

### Build learning packets

Stage 1 uses image header timestamps for one-to-one RGB/depth pairing, derives
the visible patch midpoint, finds its future central-encoder crossing, and
centres telemetry on that contact time. The defaults are a 10 ms image-pair
tolerance, a 100 ms local telemetry window, and a separate 1 second spectral
window.

```bash
ros2 run legwheel_dataset isolate_packets \
  --run-directory /path/to/run_1
```

Run Stage 2 on the directory printed by Stage 1:

```bash
ros2 run legwheel_dataset extract_features \
  --dataset /path/to/run_1/derived/CONFIGURATION_HASH
```

Or run both stages together:

```bash
ros2 run legwheel_dataset build_dataset \
  --run-directory /path/to/run_1
```

Algorithm settings can be overridden with `--processing-config`; the complete
defaults are installed from
`src/legwheel_dataset/config/postprocess_defaults.yaml`. Raw runs are never
modified or overwritten. The dependency-light output uses `.npy` image/patch
arrays, compressed `.npz` telemetry/features, JSON packet metadata, and JSONL
manifests. `quality_report.json` records every rejected packet reason.

#### Post-processing configuration reference

The default processing file is:

```yaml
schema_version: "1.0"
rgb_depth_max_delta_ms: 10.0
local_window_ms: 100.0
spectral_window_ms: 1000.0
telemetry_resample_hz: 100.0
maximum_interpolation_gap_ms: 50.0
track_point_count: 360
depth_scale_m: 0.001
minimum_plane_points: 100
plane_ransac_iterations: 200
plane_inlier_threshold_m: 0.005
minimum_spectral_valid_fraction: 0.8
```

All time settings ending in `_ms` are in milliseconds. Distance settings
ending in `_m` are in metres. These values affect derived data only; they do
not change the original MCAP recording.

##### `rgb_depth_max_delta_ms: 10.0`

This is the maximum permitted absolute difference between an RGB image header
timestamp and a depth image header timestamp. Pairing is nearest-neighbour,
one-to-one: an accepted depth image cannot be reused by another RGB image. A
pair is accepted when:

```text
abs(depth_time - rgb_time) <= rgb_depth_max_delta_ms
```

The packet's representative image time is the integer midpoint
`(rgb_time + depth_time) / 2`. Reducing this value improves temporal alignment
but rejects more image pairs. Increasing it produces more pairs but allows the
robot to move farther between the RGB and depth exposures. At `4.75 rad/s`, a
10 ms difference is `0.0475 rad` or approximately `2.72 degrees`; this is about
`4.75 mm` at a `0.1 m` wheel radius, but about `39 mm` along a `0.83 m` rig
track. The relevant distance must be chosen for the physical motion being
modelled. This setting synchronizes timestamps; it does not register depth
pixels onto the RGB pixel grid.

##### `local_window_ms: 100.0`

This is the duration of the short telemetry packet associated with a terrain
patch. It is centred on the calculated future wheel-contact time, not on the
camera timestamp. The default interval is therefore `-50 ms` through `+50 ms`
relative to contact. At `100 Hz`, both endpoints are included, so the stored
`local_values` and `local_valid` arrays contain 11 rows at
`[-50, -40, ..., 0, ..., +40, +50] ms`.

A shorter window isolates contact more tightly but can miss the complete
mechanical response. A longer window adds context but can mix the selected
patch with neighbouring terrain. At `4.75 rad/s` and a `0.1 m` wheel radius,
100 ms represents approximately `4.75 cm` of wheel-surface travel, nominally
half before and half after the central contact when speed is constant and
there is no slip.

##### `spectral_window_ms: 1000.0`

This is the separate telemetry duration used for FFT calculation. It is also
centred on the calculated contact time. The default interval is `-500 ms`
through `+500 ms`. At `100 Hz`, inclusive endpoints produce 101 rows in
`spectral_values` and `spectral_valid`.

The longer window is intentional: an 18 Hz oscillation contains only 1.8
cycles in 100 ms but approximately 18 cycles in one second. Longer windows
provide finer frequency resolution but cover more neighbouring terrain and
are harder to obtain near the start or end of a recording. With the current
101-sample default, NumPy's FFT bins are spaced by approximately
`100 / 101 = 0.9901 Hz`. `spectral_window_ms` must not be shorter than
`local_window_ms`.

##### `telemetry_resample_hz: 100.0`

Leg, wheel, rotation-encoder, and IMU messages are not normally recorded at
identical timestamps. This setting creates one common, uniformly spaced grid
for all 17 telemetry channels. At `100 Hz`, adjacent target samples are 10 ms
apart. The exact number of stored rows is:

```text
max(2, round(window_ms * telemetry_resample_hz / 1000) + 1)
```

The final `+1` includes both window endpoints. For FFTs, the largest
representable frequency is the Nyquist frequency, one half of the resampling
rate: `50 Hz` at the default setting. Increasing the rate creates larger arrays
and permits higher represented frequencies, but interpolation cannot create
information that was absent from a slower raw sensor stream. Decreasing it
reduces storage and frequency range and can lose high-frequency behaviour.

##### `maximum_interpolation_gap_ms: 50.0`

This is the largest allowed time separation between the two source samples
surrounding a resampling target. If the surrounding samples are 20 ms apart,
the default permits linear interpolation. If they are 100 ms apart, the output
cell is set to `NaN` and its validity mask is `false`. Exact source timestamps
are copied directly, and the processor does not extrapolate beyond the first
or last source sample.

Reducing the limit is stricter and exposes more dropouts as invalid data.
Increasing it fills more cells but can hide a sensor dropout by inventing a
smooth transition across a long unmeasured interval. This value also affects
whether a channel passes the FFT valid-fraction requirement below.

##### `track_point_count: 360`

This is the number of angular samples generated on each of the inner and outer
circular track boundaries. The default therefore projects 360 inner points and
360 outer points, spaced by one degree. At a radius of `0.83 m`, one degree is
approximately `1.45 cm` of arc length.

These projected boundary samples define the visible RGB/depth polygons and the
visible patch midpoint angle. Consequently, this setting indirectly affects
the calculated contact time. It does **not** set the number of reconstructed
depth points used for plane fitting; those come from all valid depth pixels
inside the polygon. More samples produce a smoother boundary and finer angular
midpoint at additional computation cost. Fewer samples produce a coarser mask
and contact-angle estimate. The minimum accepted value is 16.

##### `depth_scale_m: 0.001`

This converts integer depth-image values into metres:

```text
depth_m = raw_integer_depth * depth_scale_m
```

With the default, a raw value of `1000` becomes `1.0 m`, as expected for a
millimetre-valued `16UC1` image. This scale directly affects every reconstructed
3D point, the fitted plane offset, RANSAC distances, and plane RMSE. An
incorrect scale makes the complete 3D reconstruction metrically wrong.
Floating-point depth images such as `32FC1` are assumed to already contain
metres and are not multiplied by this value.

##### `minimum_plane_points: 100`

This is the minimum number of valid 3D depth points required before fitting is
allowed, and also the minimum number of RANSAC inliers required to accept the
best plane. It is an absolute point count, not a percentage. Raising it demands
more geometric evidence and can reject small or sparse patches. Lowering it
accepts smaller patches but makes the plane more sensitive to depth noise. The
configuration permits no value below three, because three non-collinear points
are the mathematical minimum for a plane.

##### `plane_ransac_iterations: 200`

For each iteration, deterministic RANSAC selects three depth points, constructs
a candidate plane, and counts how many other points lie within
`plane_inlier_threshold_m`. The candidate with the most inliers is retained;
mean squared inlier error breaks ties. The chosen plane is then refined with
all its inliers using singular value decomposition. The random generator uses
a fixed seed, so identical input and settings produce identical output.

More iterations increase the chance of finding the terrain plane when many
points are outliers, at a proportional CPU cost. Fewer iterations are faster
but increase the chance of accepting a poor candidate.

##### `plane_inlier_threshold_m: 0.005`

This is the maximum perpendicular point-to-plane distance for a reconstructed
depth point to count as a RANSAC inlier. The default is `0.005 m`, or `5 mm`.
A point exactly 5 mm from a candidate plane is included; a point farther away
is excluded.

A smaller threshold demands a flatter, cleaner surface and can leave fewer
than `minimum_plane_points` inliers. A larger threshold tolerates depth noise
and rough terrain but can incorrectly describe curved or uneven terrain as one
plane. It influences the final normal, along/cross slopes, inlier ratio, and
plane RMSE. It must use the same metre scale as the reconstructed depth points.

##### `minimum_spectral_valid_fraction: 0.8`

This is the minimum valid-sample fraction required separately for each
telemetry channel before its FFT is calculated. With 101 default spectral
samples, 81 valid samples pass (`81 / 101 = 0.802`) while 80 do not
(`80 / 101 = 0.792`). A passing channel has its remaining missing samples
filled by interpolation, is linearly detrended, receives a Hann window, and is
then transformed. A failing channel keeps `NaN` spectral power and has no
dominant frequency (`null` in JSON). Other channels in the same packet can
still be valid.

Increasing this fraction demands more complete telemetry and reduces the
amount of data invented by gap filling. Decreasing it produces more spectra
but makes them depend more heavily on interpolation. The valid range is greater
than zero and at most one.

The settings interact in four main groups:

- Image association: `rgb_depth_max_delta_ms`.
- Contact packet construction: `local_window_ms`,
  `telemetry_resample_hz`, and `maximum_interpolation_gap_ms`.
- Frequency analysis: `spectral_window_ms`, `telemetry_resample_hz`,
  `maximum_interpolation_gap_ms`, and `minimum_spectral_valid_fraction`.
- Terrain geometry: `track_point_count`, `depth_scale_m`,
  `minimum_plane_points`, `plane_ransac_iterations`, and
  `plane_inlier_threshold_m`.

A stricter downstream setting cannot repair an incorrect upstream scale or
association. For example, more RANSAC iterations cannot correct a wrong depth
scale, and a longer FFT window cannot recover telemetry hidden by long sensor
dropouts.

The processor currently fixes `theta_y` to zero because rotating a radially
circular local track does not change its camera projection. The source contains
an explicit TODO at this approximation so it can be removed when non-circular
or world-fixed track geometry is introduced.

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
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Post-processing visualization | Dynamic and fixed frame transforms regenerated while replaying a bag. |
| `/legwheel/visualization/kinematics_3d` | `visualization_msgs/msg/MarkerArray` | Post-processing visualization | Inner/outer track points and moving kinematic-chain origins for Foxglove. |
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
