# Gripper action interface

This document covers the **gripper-level** actions exposed by gripper wrapper packages.

The intended ownership is:

- gripper-level wrapper packages expose `/open_gripper` and `/close_gripper`
- low-level servo packages expose `/servo_control` separately

## Action types

### `gripper_msgs/action/OpenGripper`

Goal:

| Field | Type | Description |
| --- | --- | --- |
| `torque` | `float` | Requested torque or effort value. A non-zero goal overrides the node's configured control torque. |
| `use_torque_mode` | `bool` | If `true`, the driver may apply torque or current limiting. |

Result:

| Field | Type | Description |
| --- | --- | --- |
| `success` | `bool` | Indicates whether the gripper command completed successfully. |
| `message` | `string` | Driver or controller status message for the completed command. |

Feedback:

| Field | Type | Description |
| --- | --- | --- |
| `progress` | `float32` | Best-effort progress estimate from $0.0$ to $1.0$. |

### `gripper_msgs/action/CloseGripper`

Goal:

| Field | Type | Description |
| --- | --- | --- |
| `close_ratio` | `float` | Requested close amount. `1.0` closes fully, `0.3` closes part-way, and `0.0` falls back to the node's configured default close behavior. |
| `torque` | `float` | Requested torque or effort value. A non-zero goal overrides the node's configured control torque. |
| `use_torque_mode` | `bool` | If `true`, the driver may apply torque or current limiting. |

Result:

| Field | Type | Description |
| --- | --- | --- |
| `success` | `bool` | Indicates whether the gripper command completed successfully. |
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
ros2 interface show gripper_msgs/action/OpenGripper
ros2 interface show gripper_msgs/action/CloseGripper
```

Send an open gripper:

```bash
source install/setup.bash
ros2 action send_goal /open_gripper gripper_msgs/action/OpenGripper "{torque: 0.0, use_torque_mode: false}"
```

Send a close gripper:

```bash
source install/setup.bash
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close_ratio: 1.0, torque: 0.0, use_torque_mode: false}"
```

Send a partial close:

```bash
source install/setup.bash
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close_ratio: 0.3, torque: 0.0, use_torque_mode: false}"
```

Close with torque/current limiting (if supported by the active driver):

```bash
source install/setup.bash
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close_ratio: 1.0, torque: 0.5, use_torque_mode: true}"
```

## Notes on `torque`

The action goal field is still driver-specific, but both servo backends now support a configurable torque interpretation:

- Dynamixel: if `torque_per_current_unit` is calibrated in Nm, then the action goal `torque` value is also in Nm.
- Dynamixel position mode monitors measured torque and stops at `safety_torque_limit` without reopening the gripper.
- Dynamixel torque mode uses `control_torque` unless the goal sends a non-zero `torque`, and it stops once the requested torque is reached while holding position.
- Feetech: if `torque_limit_per_torque_unit` and `torque_per_current_unit` are calibrated, the action goal `torque` can also be interpreted in configured torque units.
- Feetech still uses an approximate torque mode implemented through a torque-limit register plus current monitoring, not true current-based position control.

If you want to command physical units such as Nm, calibrate the active driver so one action-unit maps consistently to the driver's measured torque estimate.
