# legwheel_can

ROS 2 Jazzy C++ package for communicating with MAB Robotics MD80 motor
controllers through the MAB CANdle-SDK. The package is arranged around a
safety-first split:

- `md80_zero_node` is for passive encoder observation and zeroing.
- `md80_impedance_node` is for read-only monitoring, gated fixed-1DOF leg
  control, and wheel velocity control on the same CAN bus.
- `legwheel_controller_node` is a Python terminal command publisher for
  knee impedance targets, gains, and motor enable/stop status.
- `wheel_controller_node` is a Python keyboard controller for publishing wheel
  velocity requests.

The default logical joints are:

| Joint | MD80 parameter | Default ID |
| --- | --- | --- |
| `hip_joint` | `hip_motor_id` | `461` |
| `knee_joint` | `knee_motor_id` | `923` |
| `wheel_joint` | `wheel_motor_id` | `0` placeholder |

The runtime node tolerates partial hardware. Missing motors are logged and
skipped. Set `wheel_motor_id` to the real MD80 ID before using wheel control.

## Files Added Or Updated

- `config/motor_ids.yaml`: maps logical hip/knee joints to MD80 CAN IDs.
- `config/motor_limits.yaml`: software position, velocity, and torque limits.
- `config/motor_config.json`: startup zero/gain defaults and runtime safety
  settings that are not ROS YAML parameters.
- `src/md80_zero_node.cpp`: passive calibration and zeroing node.
- `src/md80_impedance_node.cpp`: impedance/read-only runtime node.
- `scripts/legwheel_controller_node.py`: terminal command node for publishing
  knee impedance commands and motor enable/stop status.
- `scripts/wheel_controller_node.py`: keyboard command node for publishing wheel
  speed requests.
- `launch/md80_zero.launch.py`: launches the zeroing node with motor IDs.
- `launch/md80_impedance.launch.py`: launches the impedance node with motor IDs
  and software limits.
- `launch/launch_controller.launch.py`: launches only the Python controller node
  for use in a separate terminal.
- `launch/wheel_controller.launch.py`: launches only the Python wheel controller
  for use in a separate terminal.
- `CMakeLists.txt` and `package.xml`: build all package executables, link
  CANdle-SDK where needed, install launch/config files, and declare ROS
  dependencies.

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
enable gate and calls `md.disable()` on every connected motor. It then refreshes
encoder readings and saves them as `motor_position_on_shutdown`. No new targets
are sent while the gate is false.

When `/legwheel/motor_status` later publishes `true`, the node refreshes encoder
positions and compares each connected motor against the saved shutdown position.
If any connected motor moved by more than
`shutdown_to_startup_deviation_tolerance`, the node logs the expected position,
actual position, and deviation, then refuses to enable the motors. The default
saved shutdown positions are `0.0` rad for both hip and knee.

After a refused startup, another `/legwheel/motor_status=false` command will
still disable the motors, but it will not overwrite `motor_position_on_shutdown`.
This prevents accidentally accepting a moved pose as the new safe reference; move
the motors back within tolerance and publish `/legwheel/motor_status=true` to
clear the interlock.

Runtime control is fixed in 1DOF mode:

- Knee is configured as MD80 `IMPEDANCE` during initialization and before
  re-enable after a stop.
- Hip is configured as MD80 `POSITION_PID` during initialization and before
  re-enable after a stop.
- The command loop does not switch MD80 motion modes while sending targets.
- Knee remains a tunable spring/damper system.
- Hip mirrors knee encoder position with
  `hip_target = hip_mirror_multiplier * knee_position`; the default multiplier
  is `-0.5`.
- Hip zero/gain commands are ignored by the C++ node and rejected by the
  controller.
- Wheel is configured as MD80 `RAW_TORQUE`. `/wheel/requested_torque` is
  clamped to `wheel_torque_max_nm` and rate-limited in the C++ node before it is
  sent to the MD80.

Default launch behavior is read-only. Motors do not power on just because the
impedance node starts.

## Architecture

### CANdle/MD80 Connection

Both nodes:

1. Attach to CANdle over USB at 1 Mbps.
2. Discover MD80 controllers through CANdle-SDK.
3. Compare discovered IDs against `hip_motor_id`, `knee_motor_id`, and
   `wheel_motor_id`.
4. Initialize only the expected controllers that are present.
5. Publish joint state only for connected joints.

The logical joint mapping is fixed in code:

- `hip_motor_id` -> `hip_joint`
- `knee_motor_id` -> `knee_joint`
- `wheel_motor_id` -> `wheel_joint`

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
| `/legwheel/joint_states` | `sensor_msgs/msg/JointState` | Encoder state for connected hip/knee motors |
| `/wheel/wheel_state` | `sensor_msgs/msg/JointState` | Wheel encoder position, velocity, and torque estimate |

Subscribes:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/legwheel/motor_status` | `std_msgs/msg/Bool` | High-priority enable/panic gate. `false` disables motors immediately |
| `/legwheel/spring_zero_position` | `std_msgs/msg/Float64MultiArray` | `[hip_zero_rad, knee_zero_rad]` impedance equilibrium |
| `/legwheel/spring_constant` | `std_msgs/msg/Float64MultiArray` | `[hip_kp, knee_kp]` live impedance stiffness |
| `/legwheel/damping_constant` | `std_msgs/msg/Float64MultiArray` | `[hip_kd, knee_kd]` live impedance damping |
| `/wheel/requested_torque` | `std_msgs/msg/Float64` | Requested wheel torque in Nm |

The arrays keep two entries for compatibility with existing controller messages.
In fixed 1DOF control, only the knee entry is applied for zero/spring/damping.
Hip entries are ignored because hip position is generated from the knee encoder.

Control loop:

1. Read position, velocity, and torque from connected motors.
2. Publish `/legwheel/joint_states`.
3. If the control gate is open:
   - keep the knee in its initialization-time `IMPEDANCE` mode,
   - keep the hip in its initialization-time `POSITION_PID` mode,
   - apply knee `setImpedanceParams(kp, kd)` when gains change,
   - apply `setMaxTorque(...)` from software limits,
   - enable connected motors if not already enabled,
   - command knee zero target velocity, zero feed-forward torque, and target
     position,
   - command hip position to mirror the latest knee encoder position,
   - rate-limit and command wheel velocity if the wheel is connected.
4. If the control gate is closed, do not send target commands.

Hip mirror target:

```text
hip_target_rad = hip_mirror_multiplier * knee_encoder_position_rad
```

Hip position targets are checked against hip software limits before they are
sent. If the knee motor is missing, hip mirror commands are skipped.

Command position safety is controlled by:

```yaml
command_limit_policy: "reject"
```

Supported values:

- `"reject"`: default. Unsafe position targets are rejected and not sent.
- `"clamp"`: debugging mode. Unsafe position targets are clamped into range.

### Controller Node

Executable:

```bash
legwheel_controller_node
```

The controller node reads simple text commands from the terminal and publishes
the full ROS messages expected by the impedance node. It does not talk to
CANdle-SDK or hardware directly. When launched through `ros2 launch`, terminal
stdin may not be forwarded to the controller on every system, so the same
commands can also be sent on `/legwheel/controller_command`.

Publishes:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/legwheel/motor_status` | `std_msgs/msg/Bool` | `true` allows the impedance node to enable motors, `false` disables them |
| `/legwheel/spring_zero_position` | `std_msgs/msg/Float64MultiArray` | Current `[hip_zero_rad, knee_zero_rad]` command; only knee is applied |
| `/legwheel/spring_constant` | `std_msgs/msg/Float64MultiArray` | Current `[hip_kp, knee_kp]` command; only knee is applied |
| `/legwheel/damping_constant` | `std_msgs/msg/Float64MultiArray` | Current `[hip_kd, knee_kd]` command; only knee is applied |

Subscribes:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/legwheel/controller_command` | `std_msgs/msg/String` | Text command input, for example `e` or `knee set_spring 4.0` |

Terminal commands:

| Command | Effect |
| --- | --- |
| `e` or `enable` | Publish `/legwheel/motor_status=true` |
| `s` or `stop` | Publish `/legwheel/motor_status=false` |
| `knee set_zeropos 0.0` | Set knee spring zero position to `0.0` rad |
| `knee set_spring 4.0` | Set knee spring constant to `4.0` Nm/rad |
| `knee set_damp 5.0` | Set knee damping constant to `5.0` N/(rad/s) |
| `status` | Print the controller's current command arrays |
| `help` | Print a short command summary |

Mode switching has been removed. `mode ...`, `hip ...`, and `all ...` commands
are rejected. Use `knee ...` commands to tune the active impedance behavior; hip
target is generated from the knee encoder position.

If typing directly into the launch terminal does not produce a controller log,
send the same command through ROS:

```bash
ros2 topic pub --once /legwheel/controller_command std_msgs/msg/String "{data: 'e'}"
ros2 topic pub --once /legwheel/controller_command std_msgs/msg/String "{data: 'knee set_zeropos 0.0'}"
ros2 topic pub --once /legwheel/controller_command std_msgs/msg/String "{data: 's'}"
```

On startup, the controller publishes initial zero/gain arrays by default:

```yaml
spring_zero_position: [0.0, 0.0]
spring_constant: [4.0, 4.0]
damping_constant: [0.05, 0.05]
```

It does not enable motors on startup unless launched with
`auto_enable:=true`.

### Wheel Controller Node

Executable:

```bash
wheel_controller_node
```

The wheel controller reads keyboard input and publishes `std_msgs/msg/Float64`
commands to `/wheel/requested_torque`. Hold `w` for positive wheel torque and
`s` for negative wheel torque. Releasing the key ramps the command back to zero
over `wheel_torque_rampup_time`. Terminal key release is inferred from key-repeat
timeout, so keep the controller terminal focused while driving.

| Key | Effect |
| --- | --- |
| `w` | Ramp toward `+max_torque` |
| `s` | Ramp toward `-max_torque` |
| `+` | Increase `max_torque` by `0.1` Nm, up to the controller limit |
| `-` | Decrease `max_torque` by `0.1` Nm |

### Launch Files

`md80_impedance.launch.py` starts only the impedance node.

| Argument | Default | Meaning |
| --- | --- | --- |
| `enable_control` | `false` | Arms the impedance node. Motors still wait for `/legwheel/motor_status=true` |

`launch_controller.launch.py` starts only the Python controller node.

| Argument | Default | Meaning |
| --- | --- | --- |
| `auto_enable` | `false` | Makes the controller publish `/legwheel/motor_status=true` on startup |
| `publish_initial_commands` | `true` | Publishes initial zero, spring, and damping arrays on startup |

`wheel_controller.launch.py` starts only the Python wheel controller node.

| Argument | Default | Meaning |
| --- | --- | --- |
| `torque_limit_nm` | `8.0` | Controller-side max-torque clamp, matching the default wheel YAML limit |

For normal testing, keep `auto_enable` false and type `e` only after the
mechanism is ready.

## Parameters

### Motor IDs

Configured in `config/motor_ids.yaml`:

```yaml
legwheel_can:
  ros__parameters:
    hip_motor_id: 461
    knee_motor_id: 923
    wheel_motor_id: 0
```

`wheel_motor_id` is a placeholder by default. Set it to the real MD80 ID before
running wheel velocity control.

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

    wheel_velocity_max_rad_s: 2.0
    wheel_torque_max_nm: 8.0
```

The impedance node refuses to enable control unless all limits are finite and
valid:

- `limits_provided == true`
- position min is less than position max,
- velocity limit is positive,
- torque limit is positive.

### Motor Runtime Config

Configured in `config/motor_config.json`:

```json
{
  "shutdown_to_startup_deviation_tolerance": 0.1,
  "wheel_torque_rampup_time": 1.0,
  "default_max_torque": 3.0,

  "initial_hip_zero_position_rad": 0.0,
  "initial_knee_zero_position_rad": 0.0,

  "initial_hip_spring_constant": 4.0,
  "initial_knee_spring_constant": 4.0,

  "initial_hip_damping_constant": 0.05,
  "initial_knee_damping_constant": 0.05
}
```

The tolerance and zero positions are in radians. Wheel torque values are in Nm
and the wheel ramp time is in seconds. The spring constants are in Nm/rad, and
damping constants are in N/(rad/s). The impedance node reads these values on
startup and refuses control if they are missing or invalid.

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

### 3. Control-Enabled Fixed-1DOF Runtime

Terminal 1: launch the impedance node with control armed:

```bash
ros2 launch legwheel_can md80_impedance.launch.py enable_control:=true
```

Terminal 2: launch the Python controller:

```bash
ros2 launch legwheel_can launch_controller.launch.py
```

The controller publishes initial zero/gain arrays automatically. In the
controller terminal, tune the knee impedance behavior with commands followed by
Enter:

```bash
knee set_zeropos 0.0
knee set_spring 4.0
knee set_damp 0.05
```

The hip motor is controlled automatically as a position mirror of the knee
encoder position. `hip ...`, `all ...`, and `mode ...` commands are rejected by
the controller.

Explicitly enable motors only when ready:

```bash
e
```

Panic stop:

```bash
s
```

Typing `s` publishes `/legwheel/motor_status=false`, which disables all connected
motors immediately.

If the launch terminal does not forward input to the controller, use:

```bash
ros2 topic pub --once /legwheel/controller_command std_msgs/msg/String "{data: 'e'}"
ros2 topic pub --once /legwheel/controller_command std_msgs/msg/String "{data: 's'}"
```

### 4. Wheel Velocity Control

Set `wheel_motor_id` in `config/motor_ids.yaml`, then launch the impedance node
with control armed and enable motors through the legwheel controller:

```bash
ros2 launch legwheel_can md80_impedance.launch.py enable_control:=true
ros2 launch legwheel_can launch_controller.launch.py
```

In the legwheel controller terminal, type:

```bash
e
```

In a separate terminal, launch the wheel keyboard controller:

```bash
ros2 launch legwheel_can wheel_controller.launch.py
```

Keep that terminal focused. Hold `w` to ramp forward, hold `s` to ramp backward,
and release the key to ramp back to zero. Use `+` and `-` to adjust the saved
wheel max speed by `0.1` rad/s.

Wheel state is published on:

```bash
ros2 topic echo /wheel/wheel_state
```

## Notes

- The normal impedance node never calls `md.zero()`.
- The zeroing node never calls `md.enable()`.
- Control remains disabled if software limits are absent or invalid.
- `motor_limits.yaml` contains conservative placeholder values. Review and tune
  them before enabling control on hardware.
