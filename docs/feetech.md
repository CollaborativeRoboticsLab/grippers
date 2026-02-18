
# Feetech (STS/SCS) gripper

This workspace includes a ROS 2 **action server** for a Feetech STS/SCS-style servo-based gripper, using the driver sources vendored in `gripper_feetech/include/gripper_feetech/Feetech-STSServo`.

- Node: `gripper_feetech_action_node` (package: `gripper_feetech`)
- Launch: `gripper_ros/launch/feetech.launch.py`
- Main config: `gripper_ros/config/feetech.yaml`

This node follows the common action interface described in [Action Interface](./action_interface.md).

## Main concepts

### 1) Position commands

The Feetech driver’s primary command is `setTargetPosition(id, position, speed)`.

This node:

- reads `open_position` / `close_position`
- converts to **ticks** if `position_is_radians: true`
- commands the servo in **POSITION** mode
- polls `CURRENT_POSITION` until the target is reached (within tolerance)

### 2) “Torque mode” here means torque limiting

Feetech servos expose a torque limit register (`TORQUE_LIMIT`, default address `0x30`).

When `use_torque_mode` is enabled (either via parameter or the action goal), the node writes a torque limit before sending the position target.

Notes:

- The action goal field `torque` is treated as a **raw torque limit value** (written as a 16-bit integer to the configured register).
- If your model uses different semantics/scaling, adjust `default_torque_limit` and/or the value you send.

## Quick start

### 1) Configure

Edit `gripper_ros/config/feetech.yaml`:

- `device_name`: e.g. `/dev/ttyUSB0`
- `baudrate`: e.g. `1000000` (common for STS)
- `servo_id`: your servo ID
- `open_position` / `close_position`
- optionally `speed`, `goal_tolerance_ticks`, timeouts

### 2) Launch

```bash
ros2 launch gripper_ros feetech.launch.py
```

Override the config path if needed:

```bash
ros2 launch gripper_ros feetech.launch.py params_file:=/abs/path/to/feetech.yaml
```

## Sending actions

See `docs/action_interface.md` for action goal examples.

## Parameter reference

### Transport

- `device_name` (string): serial port path
- `baudrate` (int): baud rate parameter (note: the vendored Linux serial wrapper forces 1Mbps and ignores requested baudrate)
- `servo_id` (int): servo ID

### Targets / units

- `open_position` (float)
- `close_position` (float)
- `position_is_radians` (bool): if true, converts radians → ticks
- `ticks_per_rev` (int): used only when converting radians
- `direction` (int): +1 or -1
- `zero_offset_ticks` (int)

### Motion behavior

- `speed` (int): passed to `setTargetPosition`
- `goal_tolerance_ticks` (int)
- `motion_timeout_sec` (float)
- `poll_rate_hz` (float)

### Torque limiting

- `use_torque_mode` (bool): default torque-limit behavior
- `default_torque_limit` (float): used when goal `torque` is 0.0
- `torque_limit_register` (int): register address (default 48 = 0x30)

### Close default

- `close_default` (bool): if true, an empty `{}` close goal still closes the gripper

## Troubleshooting

- If the servo does not move, check: `device_name`, permissions, `baudrate`, and `servo_id`.
- If motion times out, increase `motion_timeout_sec` or loosen `goal_tolerance_ticks`.
- If direction is reversed, set `direction: -1`.

