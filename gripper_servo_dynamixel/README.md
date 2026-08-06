# Gripper Servo Dynamixel

This package provides utility code and ROS 2 **action server** node for direct control of a Dynamixel servo using **DynamixelSDK Protocol 2.0**.

The ros2 node follows the guidelines of [Servo-level ROS2 interface](../docs/servo/ros2-interface.md) and exposes the low-level `/servo_control` action.


## Quick start

Run the low level control node via launch file:

```bash
source install/setup.bash
ros2 launch gripper_ros dynamixel.launch.py
```

Direct servo command:

```bash
source install/setup.bash
ros2 action send_goal /servo_control control_msgs/action/GripperCommand "{command: {position: 900.0, max_effort: 0.0}}"
```

Servo command with torque/current limiting (if supported by the active driver):

```bash
source install/setup.bash
ros2 action send_goal /servo_control control_msgs/action/GripperCommand "{command: {position: 900.0, max_effort: 1.0}}"
```

For more information read [Servo-level ROS2 interface](../docs/servo/ros2-interface.md) docs.


## Position mode vs torque mode

The node supports two command styles:

- **Position mode**: writes `Goal Position`.
- **Torque mode** (implemented as current/torque control): converts the requested `command.max_effort` to `Goal Current` using `torque_per_current_unit`, then writes `Goal Current`.

When torque mode is enabled **and** a target position is also provided (open/close), the node uses **Current-based Position Control** (operating mode `5` by default on many X-series), i.e. it writes both goal current and goal position.

Important: `/servo_control` embeds `control_msgs/msg/GripperCommand`, but it is still a low-level servo API. `command.position` is interpreted in configured servo command units, and `command.max_effort` is interpreted in configured torque units. When `torque_per_current_unit` is calibrated, the node converts between those torque units and Dynamixel current counts internally.

Read the value calculation at [Notes on `Torque Behavior`](../docs/servo/parameters.md#Torque-behavior) in the servo-level parameter docs for details on how to configure a calibrated torque interpretation.

### Position units

By default, `open_position` and `close_position` are interpreted as **radians** and then converted to Dynamixel ticks using:

$$\text{ticks} = \text{zero\_offset\_ticks} + \text{direction} \cdot \Big(\theta\,[rad] \cdot \frac{\text{ticks\_per\_rev} \cdot \text{gear\_ratio}}{2\pi}\Big)$$

If you prefer to specify raw ticks directly, set:

- `position_is_radians: false`


## Parameter configuration

- File: `gripper_ros/config/servos/dynamixel.yaml`
- Contains the “site specific” choices: which motor model preset, which serial port, which servo ID, and your `open_position` / `close_position`.
- The current motor YAML uses a wildcard `/**` parameter root so both the low-level and gripper-level nodes can reuse the same motor defaults.

Read Parameter Description in [Servo parameters](../docs/servo/parameters.md) docs.