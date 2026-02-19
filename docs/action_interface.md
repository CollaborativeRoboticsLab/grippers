# Gripper action interface

This workspace standardizes gripper control around **two ROS 2 actions** (from the `gripper_msgs` package):

- `/open_gripper` (`gripper_msgs/action/OpenGripper`)
- `/close_gripper` (`gripper_msgs/action/CloseGripper`)

Multiple implementations (e.g. Dynamixel, Feetech) expose these same action names so higher-level software can stay gripper-agnostic.

## Action types

### `gripper_msgs/action/OpenGripper`

Goal:

- `torque` (float): implementation-defined “effort” value
- `use_torque_mode` (bool): if true, the driver may apply torque/current limiting

Result:

- `success` (bool)
- `message` (string)

Feedback:

- `progress` (float32): $0.0 \rightarrow 1.0$ best-effort progress estimate

### `gripper_msgs/action/CloseGripper`

Goal:

- `close` (bool): whether to close (some nodes also support `close_default` as a parameter so `{}` still closes)
- `torque` (float): implementation-defined “effort” value
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

Send an open goal:

```bash
source install/setup.bash
ros2 action send_goal /open_gripper gripper_msgs/action/OpenGripper "{torque: 0.0, use_torque_mode: false}"
```

Send a close goal:

```bash
source install/setup.bash
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close: true, torque: 0.0, use_torque_mode: false}"
```

Close with torque/current limiting (if supported by the active driver):

```bash
source install/setup.bash
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close: true, torque: 80.0, use_torque_mode: true}"
```

## Notes on `torque`

The action goal field is named `torque`, but its meaning is intentionally **driver-specific**:

- A Dynamixel-based driver will typically treat it like **Goal Current** (raw value; model-specific units).
- A Feetech STS/SCS-based driver will typically treat it like a **torque limit register value** (raw value; model-specific units).

See the driver-specific docs for how that value is used.
