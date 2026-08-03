# Servo action interface

This document covers the **low-level servo control** action exposed by the servo packages.

The intended ownership is:

- low-level servo packages expose `/servo_control`
- gripper-level wrapper packages should not expose low-level servo APIs unless explicitly needed for debugging

## Action types

### `control_msgs/action/GripperCommand`

Goal:

| Field | Type | Description |
| --- | --- | --- |
| `command` | `control_msgs/msg/GripperCommand` | Low-level servo command payload. `command.position` is mapped to the configured servo command units, and `command.max_effort` is mapped to the configured torque/current limit units. |

Result:

| Field | Type | Description |
| --- | --- | --- |
| `success` | `bool` | Indicates whether the servo command completed successfully. |
| `message` | `string` | Driver or controller status message for the completed command. |

Feedback:

| Field | Type | Description |
| --- | --- | --- |
| `progress` | `float32` | Best-effort progress estimate from $0.0$ to $1.0$. |

## CLI usage

Discover actions:

```bash
source install/setup.bash
ros2 action list
```

Inspect the interface:

```bash
source install/setup.bash
ros2 interface show control_msgs/action/GripperCommand
```

Send a direct servo command:

```bash
source install/setup.bash
ros2 action send_goal /servo_control control_msgs/action/GripperCommand "{command: {position: 900.0, max_effort: 0.0}}"
```

Servo command with torque/current limiting (if supported by the active driver):

```bash
source install/setup.bash
ros2 action send_goal /servo_control control_msgs/action/GripperCommand "{command: {position: 900.0, max_effort: 0.5}}"
```

## Notes on `command.max_effort`

The low-level action uses `control_msgs/action/GripperCommand` for consistency with the gripper-level API, but `/servo_control` is still a direct servo debugging/control API:

- `command.position` is interpreted in the node's configured servo command units, not gripper jaw width in meters.
- A nonzero `command.max_effort` value typically implies a torque-limited or current-limited position command.
- A Dynamixel-based driver interprets `command.max_effort` in the configured torque units and converts it to Goal Current internally using `torque_per_current_unit`. If that parameter is calibrated in Nm, then the value is effectively in Nm as well.
- A Feetech STS/SCS-based driver can also map `command.max_effort` into configured torque units when `torque_limit_per_torque_unit` and `torque_per_current_unit` are calibrated, but it remains an approximation built on a torque-limit register rather than true current-based position control.

See the driver-specific docs for how that value is used.
