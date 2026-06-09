
# Dynamixel gripper (Protocol 2.0)

This workspace provides a ROS 2 **action server** node that opens/closes a custom gripper driven by a Dynamixel motor using **DynamixelSDK Protocol 2.0**.

The main entrypoint is:

- Node: `gripper_dynamixel_action_node` (package: `gripper_servo_dynamixel`)

For the current articulated two-finger gripper wrapper, the gripper-level entrypoint is:

- Node: `gripper_two_fingers_node` (package: `gripper_two_fingers`)

The low-level node owns one Dynamixel servo and the common open/close action behavior.
The gripper-level wrapper owns articulation publication for `robot_state_publisher`
by publishing the mechanism `joint_states` needed to keep the TF tree connected.

For the current two-finger wrapper, the preferred articulation model is now:

- one **left finger** displacement parameter block
- one **right finger** displacement parameter block
- two dynamic support-connectivity TF bridges published by `gripper_two_fingers`
- `robot_state_publisher` handling the rest of the tree

This node follows the common action interface described in [Action Interface](./action_interface.md).

## Main concepts

### 1) Two layers of configuration

There are *two* parameter YAML layers:

1. **Main runtime config** (in `gripper_ros`)
	- File: `gripper_ros/config/dynamixel.yaml`
	- Contains the “site specific” choices: which motor model preset, which serial port, which ID, and your `open_position` / `close_position`.
  - The current motor YAML uses a wildcard `/**` parameter root so both the low-level and gripper-level nodes can reuse the same motor defaults.

2. **Motor model preset** (in `gripper_servo_dynamixel`)
  - Files: `gripper_servo_dynamixel/config/<MODEL>.yaml` (examples: `XL330.yaml`, `XM430.yaml`, `W350.yaml`)
	- Contains the “motor specific” details: control table addresses, operating mode values, and position scaling constants.

At startup the node:

1. Declares parameters with safe defaults.
2. Loads the model preset if `motor_model` or `motor_config_path` is provided.
3. Initializes the Dynamixel SDK driver using the final parameter values.

### 2) Position mode vs torque mode

The node supports two command styles:

- **Position mode**: writes `Goal Position`.
- **Torque mode** (implemented as current/torque control): writes `Goal Current`.

When torque mode is enabled **and** a target position is also provided (open/close), the node uses **Current-based Position Control** (operating mode `5` by default on many X-series), i.e. it writes both goal current and goal position.

Important: the goal field named `torque` is treated as a **raw Goal Current value** (units are motor/model specific). This is intentional so the model preset can stay generic.

### 3) Position units

By default, `open_position` and `close_position` are interpreted as **radians** and then converted to Dynamixel ticks using:

$$\text{ticks} = \text{zero\_offset\_ticks} + \text{direction} \cdot \Big(\theta\,[rad] \cdot \frac{\text{ticks\_per\_rev} \cdot \text{gear\_ratio}}{2\pi}\Big)$$

If you prefer to specify raw ticks directly, set:

- `position_is_radians: false`

### 4) Gripper-level two-finger mapping

At the gripper layer, the current preferred configuration is no longer a flat list of many mechanism joints.
Instead, use a per-finger mapping:

```yaml
gripper_left_finger:
  joint_name: gripper_planar_4
  multiplier: 0.0
  offset: 0.0

gripper_right_finger:
  joint_name: gripper_planar_5
  multiplier: 0.0
  offset: 0.0
```

Interpretation:

- the measured motor position is converted into command units
- that scalar is mapped into left/right finger support displacement with `multiplier` and `offset`
- the two support-connectivity TF bridges are then derived from those two values

For a single coupled servo, the intended physical model is usually:

- `finger_displacement ~= total_gap_change / 2`

because each finger moves approximately half the distance relative to the centerline.

## Quick start

### 1) Hardware / OS prep

- Connect the motor via a U2D2 (or equivalent USB2Serial adapter).
- Ensure your user can access the serial port (often requires `dialout` group):

```bash
sudo usermod -a -G dialout $USER
```

Log out/in after changing groups.

### 2) Set your main config

Edit `gripper_ros/config/dynamixel.yaml`:

- `motor_model`: `XL330` / `XM430` / `W350` (or your custom preset name)
- `device_name`: e.g. `/dev/ttyUSB0`
- `baudrate`: e.g. `57600` or whatever you configured on the motor
- `dxl_id`: the motor ID
- `open_position` / `close_position`: your gripper’s open/close targets

If you do not know the motor ID or baudrate yet, use the probe script before editing the config:

```bash
source install/setup.bash
python3 src/grippers/gripper_servo_dynamixel/find_id.py \
  --device /dev/ttyUSB0 \
  --baudrate-sweep 57600 115200 1000000 2000000 3000000 4000000 \
  --scan-start 0 \
  --scan-end 252 \
  --no-poll
```

### 3) Run

Launch via `gripper_ros`:

```bash
source install/setup.bash
ros2 launch gripper_ros dyanmixel.launch.py
```

Launch the gripper-level two-finger wrapper:

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_soft_two_finger.launch.py
```

That wrapper can publish:

- gripper-level `joint_states`
- two dynamic TF bridges for support connectivity

while `robot_state_publisher` handles the rest of the URDF tree.

Override the params file if needed:

```bash
source install/setup.bash
ros2 launch gripper_ros dyanmixel.launch.py params_file:=/abs/path/to/dynamixel.yaml
```

## Sending actions

See `docs/action_interface.md` for action goal examples.

## Probe and diagnostics

The repository includes a standalone probe script at `src/grippers/gripper_servo_dynamixel/find_id.py`.

Use it when you need to:

- scan an ID range
- sweep multiple baudrates
- poll common registers from a known servo

### Scan one baudrate across an ID range

```bash
python3 src/grippers/gripper_servo_dynamixel/find_id.py \
  --device /dev/ttyUSB0 \
  --baudrate 57600 \
  --scan-start 0 \
  --scan-end 20 \
  --no-poll
```

### Sweep common baudrates and IDs

```bash
python3 src/grippers/gripper_servo_dynamixel/find_id.py \
  --device /dev/ttyUSB0 \
  --common-baudrate-sweep \
  --scan-start 0 \
  --scan-end 252 \
  --no-poll
```

### Poll a known servo continuously

```bash
python3 src/grippers/gripper_servo_dynamixel/find_id.py \
  --device /dev/ttyUSB0 \
  --id 1 \
  --baudrate 57600 \
  --count 0 \
  --interval 0.5
```

The script prints `model`, `operating_mode`, `torque_enable`, `hardware_error`, `present_current`, and `present_position` when a servo is reachable.

### Interpreting common failures

- `There is no status packet!`: transmit succeeded, but no valid Dynamixel reply came back. Check power, wiring, adapter type, baudrate, protocol version, and ID.
- `Incorrect status packet!`: some bytes came back, but they were not a valid Dynamixel response. This usually points to bus noise, the wrong serial device, or a non-Dynamixel device on that port.
- Port changed from `/dev/ttyUSB0` to `/dev/ttyUSB1`: the USB serial adapter re-enumerated after reconnect/reset. Re-run the probe on the new device path.
- `Permission denied`: fix serial permissions first with the `dialout` group change described above, then log out/in before retrying.

## Parameter reference (most important)

### Config selection

- `motor_model` (string): loads `${share}/config/<motor_model>.yaml` from the `gripper_servo_dynamixel` package.
- `motor_config_path` (string): explicit YAML file path (overrides `motor_model` lookup).

### Transport

- `device_name` (string): serial port, e.g. `/dev/ttyUSB0`
- `baudrate` (int): serial baud rate
- `dxl_id` (int): motor ID

### Gripper targets

- `open_position` (float): target open position (radians by default)
- `close_position` (float): target close position (radians by default)

### Gripper-level two-finger articulation

- `publish_gripper_joint_states` (bool)
- `publish_support_connectivity_tf` (bool): publish the two support-connectivity TF bridges
- `gripper_joint_state_topic` (string)
- `gripper_joint_name_prefix` (string)
- `gripper_frame_prefix` (string)

Preferred parameters:

- `gripper_left_finger.joint_name`
- `gripper_left_finger.multiplier`
- `gripper_left_finger.offset`
- `gripper_right_finger.joint_name`
- `gripper_right_finger.multiplier`
- `gripper_right_finger.offset`

Legacy fallback parameters still supported in code:

- `gripper_joint_state_names`
- `gripper_joint_state_multipliers`
- `gripper_joint_state_offsets`

The preferred current setup is to use only:

- `gripper_planar_4`
- `gripper_planar_5`

and **not** publish `gripper_revolute_*` from the node for now.

Important caveat: if `gripper_revolute_*` are not published, `robot_state_publisher`
will not make them move automatically unless the URDF models those relationships explicitly
(for example with `mimic` or a different kinematic structure).

### Motion completion behavior

- `goal_tolerance_ticks` (int): success when `abs(target - present) <= tolerance`
- `motion_timeout_sec` (float): abort if not reached in time
- `poll_rate_hz` (float): present position polling frequency

### Torque mode

- `use_torque_mode` (bool): default torque-mode behavior if the action goal does not explicitly enable it
- `default_torque` (float): used when goal `torque` is `0.0`
- `close_default` (bool): used when close goal’s boolean flag is default-constructed / false

### Control table addresses (motor preset)

These *must* match your motor model’s control table:

- `addr_operating_mode`
- `addr_torque_enable`
- `addr_goal_current`
- `addr_goal_position`
- `addr_present_current`
- `addr_present_position`

### Operating mode values (motor preset)

- `operating_mode_current` (commonly `0`)
- `operating_mode_position` (commonly `3`)
- `operating_mode_current_based_position` (commonly `5`)

### Position scaling (motor preset)

- `position_is_radians` (bool)
- `ticks_per_rev` (int)
- `gear_ratio` (float)
- `direction` (int, usually `+1` or `-1`)
- `zero_offset_ticks` (int)

## Common setup steps / troubleshooting

- **YAML parsing errors**: YAML indentation must use spaces (tabs are invalid YAML). Keep `ros__parameters` blocks consistently indented.
- **Port open failures**: verify `device_name` and permissions (`dialout` group).
- **No response / timeouts**: likely wrong `dxl_id`, baudrate, or a control-table address mismatch.
- **Motor moves the wrong way**: set `direction: -1` or adjust your sign convention.
- **Open/close not matching physical endpoints**:
  - Use `zero_offset_ticks` to shift the reference
  - Adjust `open_position` / `close_position`
  - Consider switching to `position_is_radians: false` temporarily and tune in ticks

## Examples

### Minimal `dynamixel.yaml`

This is the smallest useful main config file. Keep indentation as **spaces** (tabs will break YAML parsing).

```yaml
gripper_dynamixel_action_node:

  ros__parameters:
    motor_model: XL330
    device_name: /dev/ttyUSB0
    baudrate: 57600
    dxl_id: 1

    open_position: 0.0
    close_position: 1.0
```

### Custom motor preset (new model file)

To add a new model preset, create a new file in `gripper_servo_dynamixel/config/`, for example:

`gripper_servo_dynamixel/config/MY_GRIPPER.yaml`

```yaml
gripper_dynamixel_action_node:

  ros__parameters:
    # Control table addresses (MUST match your motor's e-manual)
    addr_operating_mode: 11
    addr_torque_enable: 64
    addr_goal_current: 102
    addr_goal_position: 116
    addr_present_current: 126
    addr_present_position: 132

    # Operating modes (Protocol 2.0)
    operating_mode_current: 0
    operating_mode_position: 3
    operating_mode_current_based_position: 5

    # Position scaling
    position_is_radians: true
    ticks_per_rev: 4096
    gear_ratio: 1.0
    direction: 1
    zero_offset_ticks: 0
```

Then set in your main `gripper_ros/config/dynamixel.yaml`:

```yaml
gripper_dynamixel_action_node:

  ros__parameters:
    motor_model: MY_GRIPPER
    device_name: /dev/ttyUSB0
    baudrate: 57600
    dxl_id: 1
```

