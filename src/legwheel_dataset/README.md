# `legwheel_dataset`

`legwheel_dataset` is the offline-only ROS 2 package that converts one recorded
LegWheel MCAP run into self-contained, machine-learning-ready packets. It never
controls the robot and never changes the raw rosbag.

The package performs two processing stages:

1. **Stage 1 — packet isolation:** determine the valid constant-torque run,
   construct the continuous encoder-corrected rig angle, synchronize RGB and
   depth images, select the physical terrain patch, find when the wheel reaches
   that patch, and attach resampled telemetry.
2. **Stage 2 — feature extraction:** crop the images, reconstruct the selected
   depth pixels in 3D, robustly fit a plane, calculate two signed slopes, and
   calculate spectra for all 17 telemetry channels.

## Commands

Build both stages:

```bash
ros2 run legwheel_dataset build_dataset \
  --run-directory /path/to/experiment/run_1 \
  --processing-config /path/to/postprocess.yaml
```

Run the stages separately:

```bash
ros2 run legwheel_dataset isolate_packets \
  --run-directory /path/to/experiment/run_1 \
  --processing-config /path/to/postprocess.yaml

ros2 run legwheel_dataset extract_features \
  --dataset /path/to/experiment/run_1/derived/CONFIGURATION_HASH
```

Inspect what a configuration will produce without writing packets:

```bash
ros2 run legwheel_dataset validate_config \
  --run-directory /path/to/experiment/run_1 \
  --processing-config /path/to/postprocess.yaml
```

The validator reports the measured source rates, calculated local-window
duration, spectral Nyquist frequency and bin spacing, and an estimate of the
number of depth pixels in a fully visible patch.

## Required input

The `--run-directory` must contain:

```text
run_1/
├── rosbag/                                  # MCAP rosbag2 directory
├── run_config.yaml
├── metadata.json
├── experiment_config_run_1_snapshot.yaml
└── camera_driver_config_run_1_snapshot.yaml
```

The bag must contain:

- `/legwheel_rgbd/color/image_raw`
- `/legwheel_rgbd/color/camera_info`
- `/legwheel_rgbd/depth/image_raw`
- `/legwheel_rgbd/depth/camera_info`
- `/legwheel_rgbd/gyro_accel/sample`
- `/legwheel/joint_states`
- `/wheel/wheel_state`
- `/wheel/requested_torque`
- `/rotation_encoder/joint_state`
- `/rotation_encoder/ticks`

This implementation deliberately requires the snapshotted camera configuration
to contain `depth_registration: true` and `align_target_stream: COLOR`.
Unregistered depth is rejected instead of silently using the wrong RGB/depth
extrinsic transform.

## Output layout

By default, output is placed in:

```text
RUN_DIRECTORY/derived/CONFIGURATION_HASH/
```

The hash covers the experiment configuration, run configuration, and complete
post-processing configuration. An existing derived directory is never
overwritten.

```text
CONFIGURATION_HASH/
├── manifest.jsonl
├── features.jsonl                         # after Stage 2
├── provenance.json
├── quality_report.json
└── packets/
    ├── analysis_report.json
    ├── run_motion_analysis.npz
    └── e<experiment>_r<run>_theta_<angle>_<hash>/
        ├── metadata.json
        ├── geometry.npz
        ├── telemetry.npz
        ├── rgb.npy                        # complete synchronized RGB image
        ├── depth.npy                      # complete synchronized depth image
        └── stage2/
            ├── rgb_patch.npy
            ├── rgb_patch_mask.npy
            ├── depth_patch.npy
            ├── depth_patch_mask.npy
            ├── point_map.npz
            ├── features.npz
            └── features.json
```

Packet names use the experiment ID, run number, unwrapped `theta_y` at image
time rounded to three decimal places, and an eight-character deterministic
hash. The exact, unrounded value always remains in `metadata.json`.

Every packet retains the complete RGB and depth images. It can therefore be
examined after the original MCAP has been moved or archived.

## Stage 1, Pass 1: analysing the run

### 1. Find the constant-torque period

The configured positive torque magnitude comes from:

```text
run_config.yaml -> wheel_parameters.commanded_wheel_torque_nm
```

The actual command may be negative because of the motor direction convention.
The processor therefore compares magnitudes. With the default 90% criterion,
the accepted period begins at the first command satisfying:

```text
abs(requested_torque) >= 0.90 * commanded_wheel_torque_nm
```

It ends at the first later command below that threshold. Exact floating-point
equality is intentionally not used because the online ramp publishes discrete
intermediate commands and may not publish the nominal endpoint exactly.

### 2. Define `t0` and reject reverse encoder changes

Repeated central-encoder values are collapsed. `t0` is the first actual encoder
count change after the torque threshold is reached. The raw encoder angle at
that point is retained in `analysis_report.json`, while the processed angle is
rebased:

```text
theta_y(t0) = 0
```

Actual encoder differences are used, including a change of more than one tick.
Isolated changes opposite to the dominant run direction are treated as encoder
bounce and reported. A sustained reversal makes the future-patch assumption
invalid and prevents useful packet generation.

### 3. Construct continuous encoder-corrected `theta_y(t)`

The effective track radius is:

```text
r_b = beam_radius_m - wheel_width_m / 2
```

For every pair of accepted encoder changes `i` and `i+1`, the predicted rig
angle is:

```text
predicted_delta_theta_i = integral(
    wheel_velocity_rad_s *
    (wheel_radius_m + grouser_effective_height_m) / r_b
) dt
```

The interval correction factor is:

```text
k_i = encoder_delta_theta_i / predicted_delta_theta_i
```

Within that interval:

```text
theta_y(t) = theta_y(t_i) + k_i * integral(t_i to t)(
    wheel_velocity_rad_s *
    (wheel_radius_m + grouser_effective_height_m) / r_b
) dt
```

All signs are preserved. If the wheel and central encoder use opposite positive
directions, `k_i` is negative and corrects that convention. No arbitrary hard
limit is applied to `k_i`; its minimum, maximum and mean are written to the run
analysis report. A zero predicted interval or an unbridgeable wheel-telemetry
gap is an error.

Wheel velocity is linearly interpolated and integrated with the trapezoidal
rule. The resulting continuous function exactly matches the central encoder at
every accepted tick endpoint.

### 4. Calculate average speeds and the local window

The reported wheel speed is the time-weighted mean absolute velocity:

```text
omega_avg = integral(abs(wheel_velocity)) dt / run_duration
```

Minimum, maximum, median and standard deviation are also reported.

The local telemetry duration uses the corrected average rig angular speed, so
it includes measured slip:

```text
average_rig_speed = abs(theta_y(end) - theta_y(t0)) / run_duration
local_window_ms = alpha_sp_rad / average_rig_speed * 1000
```

This duration is constant within one run but can differ between runs. Local
arrays consequently have variable lengths between runs. Their samples remain
on a real-time grid at `telemetry_resample_hz`; downstream model preparation
must pad or otherwise batch them deliberately.

### 5. Measure acquisition rates

The processor calculates actual timestamp rates for the leg, wheel, central
encoder and IMU streams. `packets/analysis_report.json` records sample count,
median rate, median period, 95th-percentile period, and minimum/maximum interval
rates.

Packet isolation stops if `telemetry_resample_hz` exceeds a stream's measured
median rate by more than `source_rate_tolerance_fraction`. Upsampling cannot
create information that was not measured.

Important: the current Arduino firmware publishes central-encoder counts at
20 Hz. Therefore a strict all-stream common grid above approximately 20 Hz will
fail this guard even if motor and IMU streams are faster. This also means that
the encoder channel alone cannot resolve an 18 Hz signal. Change the acquisition
rate or revise the common-grid policy deliberately; do not hide this limitation
with interpolation.

## Stage 1, Pass 2: creating packets

### 1. Synchronize images

RGB and depth images are paired one-to-one using the nearest unused depth
timestamp within `rgb_depth_max_delta_ms`. The packet image time is the integer
midpoint of the two sensor timestamps.

Only images at or after `t0` and before the last complete encoder interval are
eligible. The first eligible valid pair is kept. Later pairs must be separated
from the last saved pair by at least:

```text
1 / maximum_packet_saving_frequency_hz
```

Rejected candidates do not consume this spacing.

### 2. Define the physical terrain patch

`alpha_off_rad` is a positive distance ahead. The measured travel direction is
applied automatically:

```text
patch_centre = theta_y(image_time) + travel_direction * alpha_off_rad
```

The radial limits are:

```text
inner_radius = beam_radius_m - wheel_width_m / 2
outer_radius = beam_radius_m + wheel_width_m / 2
```

The angular limits are:

```text
patch_start = patch_centre - alpha_sp_rad / 2
patch_end   = patch_centre + alpha_sp_rad / 2
```

`track_point_count` sets the full-circle sampling density. Exact patch start and
end angles are always inserted, so the requested patch width is never rounded
to that grid.

The RGB and depth projections each use their own image timestamp, knee angle,
`theta_p`, `CameraInfo`, image frame, and camera pose. A patch is accepted only
when its complete inner boundary, outer boundary and angular endpoints are in
front of the camera and inside both images. Partial patches are rejected and
listed by timestamp and reason in `packets/analysis_report.json`.

### 3. Find when the wheel reaches the photographed patch

The contact timestamp is the first future solution of:

```text
theta_y(contact_time) = patch_centre
```

It is found by bracketing the target with corrected encoder intervals and then
bisection on continuous `theta_y(t)`. It is not snapped to a telemetry sample,
so no angular matching tolerance is required.

### 4. Attach telemetry

Both telemetry arrays are centred on `contact_time`:

- `local_values`: calculated patch-traversal duration;
- `spectral_values`: fixed `spectral_window_ms` duration.

All 17 columns use the same grid and order stored in packet metadata:

- hip position, velocity and effort;
- knee position, velocity and effort;
- wheel position, velocity and effort;
- central encoder position and velocity;
- IMU angular velocity X/Y/Z;
- IMU linear acceleration X/Y/Z.

The complete local window must remain inside the corrected run and have source
timestamp coverage without an excessive gap. Spectral cells outside the valid
run remain invalid and are subject to the Stage 2 valid-fraction rule. Ramp-up
and ramp-down telemetry are never introduced.

`telemetry.npz` saves values, per-cell validity masks and nanosecond times
relative to contact. `geometry.npz` saves bit-packed image masks, exact patch
polygons, 3D physical boundaries, and full-track overlay points.

## Stage 2: feature extraction

### RGB and depth patches

The complete images are loaded from the packet. Each bit-packed mask is
restored, and the smallest bounding crop containing the physical patch is
saved. Pixels outside the polygon are zeroed, and the separate crop mask must
be used so a genuine zero-valued pixel is not confused with padding.

### Depth point map

Every finite, positive depth pixel inside the mask is converted to metres.
Integer depth uses `depth_scale_m`; floating-point depth is treated as metres.
Pixels are undistorted using the selected depth `CameraInfo`, back-projected to
3D, and transformed from the registered camera frame into `legwheel_base`.

`point_map.npz` contains:

- `points_base_m`: `N x 3` metric points;
- `pixel_uv`: `N x 2` source depth-pixel coordinates;
- `depth_m`: the `N` original metric depth measurements;
- `ransac_inlier_mask`: which points support the accepted plane.

### RANSAC and plane refinement

RANSAC samples three points per iteration and counts points whose perpendicular
distance is at most `plane_inlier_threshold_m`. The best candidate has the most
inliers, with mean squared error as its tie-breaker. A fixed random seed makes
the result reproducible.

At least `minimum_plane_points` valid points and the same number of inliers are
required. The default is 40. The accepted inliers are refined by singular value
decomposition, and the normal is oriented so its base-frame Z component is
non-negative. `plane_rmse_m` is the root-mean-square inlier residual.

### Signed slopes

At patch-centre angle `theta_patch`, the radially outward unit vector is:

```text
e_r = [cos(theta_patch), sin(theta_patch), 0]
```

The direction-of-travel tangent is:

```text
e_t = travel_direction * [-sin(theta_patch), cos(theta_patch), 0]
```

For upward-oriented plane normal `n`:

```text
along_slope = atan2(-dot(n, e_t), n_z)
cross_slope = atan2(-dot(n, e_r), n_z)
```

Positive along-slope means terrain rises in the direction of travel. Positive
cross-slope means terrain rises radially away from the rig centre. Both radians
and degrees are saved.

### FFT processing

All 17 telemetry channels receive a separate FFT. A channel must first satisfy
`minimum_spectral_valid_fraction`. Missing samples are filled only for short,
internal gaps allowed by `maximum_interpolation_gap_ms`; leading, trailing and
long gaps invalidate that channel.

Each accepted channel is linearly detrended, multiplied by a Hann window, and
converted to a one-sided power spectral density. Zero hertz is excluded from
peak detection. The two highest distinct local maxima are saved, ordered by
power, along with their powers and the complete spectrum. A missing second peak
is represented by `null` in JSON and `NaN` in NumPy output.

FFT properties are:

```text
Nyquist frequency = telemetry_resample_hz / 2
bin spacing       = telemetry_resample_hz / spectral_sample_count
```

The bin spacing is controlled by `spectral_window_ms`, not by the local patch
duration.

## Configuration reference

The installed defaults are:

```yaml
schema_version: "2.0"
rgb_depth_max_delta_ms: 10.0
spectral_window_ms: 1000.0
telemetry_resample_hz: 100.0
maximum_interpolation_gap_ms: 50.0
maximum_packet_saving_frequency_hz: 10.0
alpha_off_rad: 0.35
alpha_sp_rad: 0.10
grouser_effective_height_m: 0.0
minimum_target_torque_fraction: 0.90
source_rate_tolerance_fraction: 0.05
track_point_count: 360
depth_scale_m: 0.001
minimum_plane_points: 40
plane_ransac_iterations: 200
plane_inlier_threshold_m: 0.005
minimum_spectral_valid_fraction: 0.8
```

`alpha_off_rad` and `alpha_sp_rad` are initial processing defaults, not measured
rig constants. Validate them against the actual camera view and desired physical
patch before generating a research dataset.

`grouser_effective_height_m` intentionally defaults to zero. A source-code TODO
marks that a geometry- and experiment-based estimator should replace this
manual correction in future.

## Foxglove visualization

Play the original bag with clock publication:

```bash
ros2 bag play /path/to/run_1/rosbag --clock
```

In another sourced terminal:

```bash
ros2 run legwheel_dataset visualise_stream \
  --dataset /path/to/run_1/derived/CONFIGURATION_HASH
```

The node follows `/clock`, including backward seeks, and publishes:

- `/legwheel/dataset/rgb_annotations`
- `/legwheel/dataset/depth_annotations`
- `/legwheel/dataset/geometry`
- `/legwheel/dataset/slope_text`
- derived moving transforms on `/tf`

The image annotations contain the selected patch and visible inner/outer track
points. The 3D markers contain the circular track, physical patch boundaries,
depth-plane inliers, fitted plane outline, plane normal, direction-of-travel
arrow, radially-outward arrow, and slope text.

Derived TF child names start with `dataset_` so they do not compete with frames
recorded in the original bag. Set the Foxglove 3D display frame to
`legwheel_base`.

## Dataset splitting

Every packet stores an unwrapped `theta_y` and `lap_index`. Repeated loops can
observe the same physical terrain. Do not randomly split packets from one run
between training and evaluation; split by complete run, experiment, or physical
terrain setup to avoid leakage.
