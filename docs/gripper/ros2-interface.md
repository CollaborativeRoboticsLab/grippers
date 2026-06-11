# Gripper action interface

This document covers the **gripper-level** actions exposed by gripper wrapper packages.

The intended ownership is:

- gripper-level wrapper packages expose `/open_gripper` and `/close_gripper`
- low-level servo packages expose `/servo_control` separately

## Action types

### `gripper_msgs/action/OpenGripper`

Goal:

- `torque` (float): requested torque/effort value; a non-zero goal overrides the node's configured control torque
- `use_torque_mode` (bool): if true, the driver may apply torque/current limiting

Result:

- `success` (bool)
- `message` (string)

Feedback:

- `progress` (float32): $0.0 \rightarrow 1.0$ best-effort progress estimate

### `gripper_msgs/action/CloseGripper`

Goal:

- `close` (bool): whether to close (some nodes also support `close_default` as a parameter so `{}` still closes)
- `torque` (float): requested torque/effort value; a non-zero goal overrides the node's configured control torque
- `use_torque_mode` (bool)

Result:

- `success` (bool)
- `message` (string)

Feedback:

- `progress` (float32)

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
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close: true, torque: 0.0, use_torque_mode: false}"
```

Close with torque/current limiting (if supported by the active driver):

```bash
source install/setup.bash
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close: true, torque: 0.5, use_torque_mode: true}"
```

## Notes on `torque`

The action goal field is still driver-specific, but both servo backends now support a configurable torque interpretation:

- Dynamixel: if `torque_per_current_unit` is calibrated in Nm, then the action goal `torque` value is also in Nm.
- Dynamixel position mode monitors measured torque and stops at `safety_torque_limit` without reopening the gripper.
- Dynamixel torque mode uses `control_torque` unless the goal sends a non-zero `torque`, and it stops once the requested torque is reached while holding position.
- Feetech: if `torque_limit_per_torque_unit` and `torque_per_current_unit` are calibrated, the action goal `torque` can also be interpreted in configured torque units.
- Feetech still uses an approximate torque mode implemented through a torque-limit register plus current monitoring, not true current-based position control.

If you want to command physical units such as Nm, calibrate the active driver so one action-unit maps consistently to the driver's measured torque estimate.
