# Camera Telemetry Preflight Checks

This tutorial adds camera health checks to `ExperimentRunnerNode` without
changing its state-machine model. The experiment remains in `PREFLIGHT`, with
motors disabled, until it has recently received an RGB image, a depth image,
and an IMU message.

The camera integration publishes the following LegWheel-facing topics:

| Stream | Topic | ROS message type |
| --- | --- | --- |
| RGB image | `/camera/rgbd/rgb/image_raw` | `sensor_msgs/msg/Image` |
| Depth image | `/camera/rgbd/depth/image_raw` | `sensor_msgs/msg/Image` |
| IMU | `/camera/imu` | `sensor_msgs/msg/Imu` |

Start the camera before starting an experiment:

```bash
ros2 launch legwheel_rgbd gemini_336.launch.py
```

The `gemini_336_camera_node` publishes these streams with
`qos_profile_sensor_data`. The experiment runner must request the same
profile; a reliable subscription will not connect to this best-effort stream.

## Why Use Message Receipt Time

Preflight must demonstrate that live messages arrive, rather than that a topic
name exists in the ROS graph. `ros2 topic list` includes topics created by
subscribers, so it cannot prove the camera publishes frames.

Record `time.monotonic()` in each callback and use it to check freshness. Do
not calculate freshness from `message.header.stamp`: camera and host clocks can
have different time domains. This follows the existing leg, wheel, and encoder
telemetry checks in `ExperimentRunnerNode`.

## 1. Import the Message Types

In `src/legwheel_experiments/legwheel_experiments/experiment_runner_node.py`,
replace the current sensor-message import with:

```python
from sensor_msgs.msg import Image, Imu, JointState
```

`JointState` is correct for the leg, wheel, and encoder. The camera streams use
`Image` and `Imu`; subscribing to `/camera/imu` as `JointState` creates a ROS
message-type mismatch and receives no data.

## 2. Add Camera Telemetry State

In `ExperimentRunnerNode.__init__`, next to the existing latest-telemetry
fields, add one message and one receive timestamp for each required stream:

```python
self._latest_camera_rgb = None
self._latest_camera_depth = None
self._latest_camera_imu = None

self._camera_rgb_received_at_s = None
self._camera_depth_received_at_s = None
self._camera_imu_received_at_s = None

# Allow some scheduling margin while still detecting a stopped camera quickly.
self._maximum_camera_image_age_s = 1.0
self._maximum_camera_imu_age_s = 0.5
```

The values are deliberately more forgiving than the 30 Hz RGB-D and 100 Hz IMU
defaults. They should become parameters when the experiment framework supports
deployment-specific preflight settings.

## 3. Add Camera Subscriptions

Create the subscriptions near the existing leg, wheel, and encoder
subscriptions:

```python
self._camera_rgb_subscription = self.create_subscription(
    Image,
    "/camera/rgbd/rgb/image_raw",
    self._camera_rgb_callback,
    qos_profile_sensor_data,
)

self._camera_depth_subscription = self.create_subscription(
    Image,
    "/camera/rgbd/depth/image_raw",
    self._camera_depth_callback,
    qos_profile_sensor_data,
)

self._camera_imu_subscription = self.create_subscription(
    Imu,
    "/camera/imu",
    self._camera_imu_callback,
    qos_profile_sensor_data,
)
```

Use the project-facing `/camera/...` topics, not the vendor topics under
`/legwheel_rgbd`. This keeps experiments independent of the camera driver.

## 4. Record Camera Callbacks

Add these callbacks beside `_leg_state_callback`:

```python
def _camera_rgb_callback(self, message: Image) -> None:
    self._latest_camera_rgb = message
    self._camera_rgb_received_at_s = time.monotonic()

def _camera_depth_callback(self, message: Image) -> None:
    self._latest_camera_depth = message
    self._camera_depth_received_at_s = time.monotonic()

def _camera_imu_callback(self, message: Imu) -> None:
    self._latest_camera_imu = message
    self._camera_imu_received_at_s = time.monotonic()
```

No image processing is needed for preflight. A valid ROS message received on
each stream demonstrates that the camera bridge is active and producing data.

## 5. Add Helper Methods

Add the following methods in the telemetry-helper section. They mirror the
existing `has_*_state()` and `*_state_age_s()` pattern:

```python
def has_camera_rgb(self) -> bool:
    return self._latest_camera_rgb is not None

def has_camera_depth(self) -> bool:
    return self._latest_camera_depth is not None

def has_camera_imu(self) -> bool:
    return self._latest_camera_imu is not None

def camera_rgb_age_s(self) -> float:
    if self._camera_rgb_received_at_s is None:
        return float("inf")
    return time.monotonic() - self._camera_rgb_received_at_s

def camera_depth_age_s(self) -> float:
    if self._camera_depth_received_at_s is None:
        return float("inf")
    return time.monotonic() - self._camera_depth_received_at_s

def camera_imu_age_s(self) -> float:
    if self._camera_imu_received_at_s is None:
        return float("inf")
    return time.monotonic() - self._camera_imu_received_at_s
```

Returning infinity before the first callback means freshness checks fail safely.
The separate received checks below provide the clearer user-facing failure
reason first.

## 6. Gate Preflight on Camera Health

Append these entries to `_preflight_checks()`, after the existing
telemetry-received checks and before the existing freshness checks:

```python
(
    "Camera RGB telemetry received",
    self.has_camera_rgb(),
    "Waiting for camera RGB telemetry",
),
(
    "Camera depth telemetry received",
    self.has_camera_depth(),
    "Waiting for camera depth telemetry",
),
(
    "Camera IMU telemetry received",
    self.has_camera_imu(),
    "Waiting for camera IMU telemetry",
),
(
    "Camera RGB telemetry is fresh",
    self.camera_rgb_age_s() <= self._maximum_camera_image_age_s,
    "Camera RGB telemetry is stale",
),
(
    "Camera depth telemetry is fresh",
    self.camera_depth_age_s() <= self._maximum_camera_image_age_s,
    "Camera depth telemetry is stale",
),
(
    "Camera IMU telemetry is fresh",
    self.camera_imu_age_s() <= self._maximum_camera_imu_age_s,
    "Camera IMU telemetry is stale",
),
```

`_get_preflight_failure_reason()` reports the first failed tuple, so this order
makes command-line output actionable: it first says which stream has never
arrived, then reports staleness only after all three have been received.

Do not use `get_publisher_count()` as the safety condition. A discovered
publisher can be inactive, publishing an incompatible type, or unable to
produce frames. Receipt and freshness checks validate useful behavior.

## 7. Build and Test

Build the changed experiment package:

```bash
colcon build --packages-select legwheel_experiments --symlink-install
source install/setup.bash
```

Before starting the experiment CLI, verify the camera streams independently:

```bash
ros2 topic hz /camera/rgbd/rgb/image_raw
ros2 topic hz /camera/rgbd/depth/image_raw
ros2 topic hz /camera/imu
ros2 topic info -v /camera/imu
```

Then launch the experiment runner. With the camera operating, the preflight log
will print one `PASS` line for each camera received/fresh check and transition
to `WAITING_FOR_ARM`. Stop the camera launch while preflight is active to check
that it remains safe and eventually reports a stale camera stream.

## Follow-up Improvements

- Declare camera topic names and maximum ages as ROS parameters instead of
  hard-coding them in the runner.
- Add a camera-required flag to the experiment YAML so experiments that do not
  record RGB-D data do not wait for a camera.
- Record latest camera message timestamps in run metadata for traceability.
- Add a unit test that invokes callbacks and advances a mocked monotonic clock,
  asserting the relevant preflight tuple changes from received to stale.
