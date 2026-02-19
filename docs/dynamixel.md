
# Dynamixel gripper (Protocol 2.0)

This workspace provides a ROS 2 **action server** node that opens/closes a custom gripper driven by a Dynamixel motor using **DynamixelSDK Protocol 2.0**.

The main entrypoint is:

- Node: `gripper_dynamixel_action_node` (package: `gripper_dynamixel`)

This node follows the common action interface described in [Action Interface](./action_interface.md).

## Main concepts

### 1) Two layers of configuration

There are *two* parameter YAML layers:

1. **Main runtime config** (in `gripper_ros`)
	- File: `gripper_ros/config/dynamixel.yaml`
	- Contains the “site specific” choices: which motor model preset, which serial port, which ID, and your `open_position` / `close_position`.

2. **Motor model preset** (in `gripper_dynamixel`)
	- Files: `gripper_dynamixel/config/<MODEL>.yaml` (examples: `XL330.yaml`, `XM430.yaml`, `W350.yaml`)
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

### 3) Run

Launch via `gripper_ros`:

```bash
source install/setup.bash
ros2 launch gripper_ros dyanmixel.launch.py
```

Override the params file if needed:

```bash
source install/setup.bash
ros2 launch gripper_ros dyanmixel.launch.py params_file:=/abs/path/to/dynamixel.yaml
```

## Sending actions

See `docs/action_interface.md` for action goal examples.

## Parameter reference (most important)

### Config selection

- `motor_model` (string): loads `${share}/config/<motor_model>.yaml` from the `gripper_dynamixel` package.
- `motor_config_path` (string): explicit YAML file path (overrides `motor_model` lookup).

### Transport

- `device_name` (string): serial port, e.g. `/dev/ttyUSB0`
- `baudrate` (int): serial baud rate
- `dxl_id` (int): motor ID

### Gripper targets

- `open_position` (float): target open position (radians by default)
- `close_position` (float): target close position (radians by default)

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

To add a new model preset, create a new file in `gripper_dynamixel/config/`, for example:

`gripper_dynamixel/config/MY_GRIPPER.yaml`

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

