# legwheel_can

ROS 2 Jazzy C++ package for communicating with two MAB Robotics MD80 motor
controllers through the MAB CANdle-SDK. The package is arranged around a
safety-first split:

- `md80_zero_node` is for passive encoder observation and zeroing.
- `md80_impedance_node` is for read-only monitoring or gated impedance control.

The default logical joints are:

| Joint | MD80 parameter | Default ID |
| --- | --- | --- |
| `hip_joint` | `hip_motor_id` | `461` |
| `knee_joint` | `knee_motor_id` | `923` |

Both nodes tolerate partial hardware: hip only, knee only, both motors, or no
expected motors. Missing motors are logged and skipped.

## Files Added Or Updated

- `config/motor_ids.yaml`: maps logical hip/knee joints to MD80 CAN IDs.
- `config/motor_limits.yaml`: software position, velocity, and torque limits.
- `src/md80_zero_node.cpp`: passive calibration and zeroing node.
- `src/md80_impedance_node.cpp`: impedance/read-only runtime node.
- `launch/md80_zero.launch.py`: launches the zeroing node with motor IDs.
- `launch/md80_impedance.launch.py`: launches the impedance node with motor IDs
  and software limits.
- `CMakeLists.txt` and `package.xml`: build both nodes, link CANdle-SDK, install
  launch/config files, and declare ROS dependencies.

## Safety Model

Firmware-level MD80 limits are managed manually with `candletool`. This package
does not write firmware limits.

The zeroing node is intentionally passive:

- It never calls `md.enable()`.
- It never commands position, velocity, torque, or impedance targets.
- It disables connected motors on startup and shutdown as a conservative guard.
- It only reads encoder state and calls `md.zero()` when a zero service is
  requested.

The impedance node is gated by three independent conditions before it enables or
commands a motor:

1. Launch/parameter `enable_control` must be `true`.
2. `/legwheel/motor_status` must publish `true`.
3. `limits_provided` must be `true` and all software limits must validate.

If `/legwheel/motor_status` publishes `false`, the node immediately drops the
enable gate and calls `md.disable()` on every connected motor. No new targets are
sent while the gate is false.

Default launch behavior is read-only. Motors do not power on just because the
impedance node starts.

## Architecture

### CANdle/MD80 Connection

Both nodes:

1. Attach to CANdle over USB at 1 Mbps.
2. Discover MD80 controllers through CANdle-SDK.
3. Compare discovered IDs against `hip_motor_id` and `knee_motor_id`.
4. Initialize only the expected controllers that are present.
5. Publish joint state only for connected joints.

The logical joint mapping is fixed in code:

- `hip_motor_id` -> `hip_joint`
- `knee_motor_id` -> `knee_joint`

### Zeroing Node

Executable:

```bash
md80_zero_node
```

Publishes:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/legwheel/joint_states` | `sensor_msgs/msg/JointState` | Current encoder position, velocity, and torque estimate for connected motors |

Services:

| Service | Type | Action |
| --- | --- | --- |
| `/legwheel/zero_hip_motor` | `std_srvs/srv/Trigger` | Calls `md.zero()` on the hip MD80 only |
| `/legwheel/zero_knee_motor` | `std_srvs/srv/Trigger` | Calls `md.zero()` on the knee MD80 only |

If the requested motor is missing, the service returns `success=false` with a
human-readable message.

### Impedance Node

Executable:

```bash
md80_impedance_node
```

Publishes:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/legwheel/joint_states` | `sensor_msgs/msg/JointState` | Encoder state for connected motors |

Subscribes:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/legwheel/motor_status` | `std_msgs/msg/Bool` | High-priority enable/panic gate. `false` disables motors immediately |
| `/legwheel/spring_zero_position` | `std_msgs/msg/Float64MultiArray` | `[hip_zero_rad, knee_zero_rad]` impedance equilibrium |
| `/legwheel/spring_constant` | `std_msgs/msg/Float64MultiArray` | `[hip_kp, knee_kp]` live impedance stiffness |
| `/legwheel/damping_constant` | `std_msgs/msg/Float64MultiArray` | `[hip_kd, knee_kd]` live impedance damping |

Control loop:

1. Read position, velocity, and torque from connected motors.
2. Publish `/legwheel/joint_states`.
3. If the control gate is open:
   - set MD80 mode to `IMPEDANCE` if needed,
   - apply `setImpedanceParams(kp, kd)`,
   - apply `setMaxTorque(...)` from software limits,
   - enable the motor if not already enabled,
   - command zero target velocity, zero feed-forward torque, and target position.
4. If the control gate is closed, do not send target commands.

Command position safety is controlled by:

```yaml
command_limit_policy: "reject"
```

Supported values:

- `"reject"`: default. Unsafe position targets are rejected and not sent.
- `"clamp"`: debugging mode. Unsafe position targets are clamped into range.

## Parameters

### Motor IDs

Configured in `config/motor_ids.yaml`:

```yaml
legwheel_can:
  ros__parameters:
    hip_motor_id: 461
    knee_motor_id: 923
```

### Software Limits

Configured in `config/motor_limits.yaml`:

```yaml
legwheel_can:
  ros__parameters:
    limits_provided: true

    hip_position_min_rad: -0.5
    hip_position_max_rad: 0.5
    hip_velocity_max_rad_s: 1.0
    hip_torque_max_nm: 2.0

    knee_position_min_rad: -0.5
    knee_position_max_rad: 0.5
    knee_velocity_max_rad_s: 1.0
    knee_torque_max_nm: 2.0
```

The impedance node refuses to enable control unless all limits are finite and
valid:

- `limits_provided == true`
- position min is less than position max,
- velocity limit is positive,
- torque limit is positive.

## Build

From the workspace root:

```bash
colcon build --packages-select legwheel_can --symlink-install
source install/setup.bash
```

The package expects the CANdle-SDK submodule at:

```bash
src/third_party/CANdle-SDK
```

If it is missing, initialize it:

```bash
git submodule update --init src/third_party/CANdle-SDK
```

## Usage Guide

### 1. Passive Zeroing

Start the zeroing node:

```bash
ros2 launch legwheel_can md80_zero.launch.py
```

Watch encoder data while moving the mechanism by hand:

```bash
ros2 topic echo /legwheel/joint_states
```

Move the hip to its desired physical zero pose, then call:

```bash
ros2 service call /legwheel/zero_hip_motor std_srvs/srv/Trigger {}
```

Move the knee to its desired physical zero pose, then call:

```bash
ros2 service call /legwheel/zero_knee_motor std_srvs/srv/Trigger {}
```

The motors should remain unpowered for this whole workflow.

### 2. Read-Only Runtime Monitoring

Start the impedance node without enabling control:

```bash
ros2 launch legwheel_can md80_impedance.launch.py
```

This publishes encoder state but does not enable motors.

### 3. Control-Enabled Impedance Runtime

Launch with control armed:

```bash
ros2 launch legwheel_can md80_impedance.launch.py enable_control:=true
```

Then a separate controller or teleop node must explicitly open the motor gate:

```bash
ros2 topic pub --once /legwheel/motor_status std_msgs/msg/Bool "{data: true}"
```

Set equilibrium and gains:

```bash
ros2 topic pub --once /legwheel/spring_zero_position std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
ros2 topic pub --once /legwheel/spring_constant std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0]}"
ros2 topic pub --once /legwheel/damping_constant std_msgs/msg/Float64MultiArray "{data: [0.05, 0.05]}"
```

Panic stop:

```bash
ros2 topic pub --once /legwheel/motor_status std_msgs/msg/Bool "{data: false}"
```

Publishing `false` disables all connected motors immediately.

## Notes

- The normal impedance node never calls `md.zero()`.
- The zeroing node never calls `md.enable()`.
- Control remains disabled if software limits are absent or invalid.
- `motor_limits.yaml` contains conservative placeholder values. Review and tune
  them before enabling control on hardware.
