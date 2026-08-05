# Gripper Parameter reference

## Gripper-level common parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `motor_model` | `string` | Used to find the relevant servo config for the underlying motor model. This is used to override specific parameters from servo config. |
| `publish_gripper_joint_states` | `bool` | Whether to publish the gripper joint states on `gripper_joint_state_topic`. If `false`, the node still publishes the underlying servo joint states but not the gripper-level ones. |
| `gripper_joint_state_topic` | `string` | Topic on which to publish the gripper joint states. |
| `gripper_joint_name_prefix` | `string` | Prefix for the gripper joint names. |
| `gripper_joint_state_rate_hz` | `float` | Rate at which to publish the gripper joint states. |
| `gripper_open_width` | `float` | Jaw opening in meters represented by the configured servo `open_position`. |
| `gripper_closed_width` | `float` | Jaw opening in meters represented by the configured servo `close_position`. |
| `gripper_position_tolerance` | `float` | Position tolerance in meters for `GripperCommand` feedback/result reporting. |

These top-level parameters describe the gripper in gripper-space rather than servo-space.

- `gripper_open_width` and `gripper_closed_width` are the public values used by the gripper action interface.
- Servo-specific values such as raw Dynamixel counts still live under the selected motor-model section.

## Two-finger articulation parameters

The current preferred two-finger model publishes one slider joint per finger side.

| Parameter | Type | Description |
| --- | --- | --- |
| `gripper_left_finger.joint_name` | `string` | Joint name published for the left finger side. |
| `gripper_left_finger.joint_open_position` | `float` | Left finger joint value that corresponds to `gripper_open_width`. |
| `gripper_left_finger.joint_close_position` | `float` | Left finger joint value that corresponds to `gripper_closed_width`. |
| `gripper_right_finger.joint_name` | `string` | Joint name published for the right finger side. |
| `gripper_right_finger.joint_open_position` | `float` | Right finger joint value that corresponds to `gripper_open_width`. |
| `gripper_right_finger.joint_close_position` | `float` | Right finger joint value that corresponds to `gripper_closed_width`. |

For the current soft two-finger setup:

- `gripper_open_width` is the total jaw opening.
- Each finger joint usually represents one side of that motion, so the open joint displacement is typically about half of the total width change.
- In the current Dynamixel overlay, `gripper_open_width: 0.09` matches `joint_open_position: -0.045` on both finger joints.

## Motor-model section

The selected `motor_model` key points to a nested parameter block for the underlying actuator.

Typical motor-model parameters include:

| Parameter | Type | Description |
| --- | --- | --- |
| `<motor_model>.device_name` | `string` | Serial device path for the motor controller. |
| `<motor_model>.baudrate` | `int` | Serial baudrate used by the motor controller. |
| `<motor_model>.servo_id` | `int` | Servo ID on the bus. |
| `<motor_model>.position_is_radians` | `bool` | Whether raw open and close positions are expressed in radians instead of device units. |
| `<motor_model>.open_position` | `float` | Underlying servo-space value for the fully open gripper state. |
| `<motor_model>.close_position` | `float` | Underlying servo-space value for the fully closed gripper state. |
| `<motor_model>.control_torque` | `float` | Default torque/current limit used during active control where supported. |
| `<motor_model>.use_torque_mode` | `bool` | Whether to use the actuator's torque/current-control path when available. |
| `<motor_model>.default_torque` | `float` | Default resting torque/current setting. |
| `<motor_model>.safety_torque_limit` | `float` | Safety limit used to clamp requested torque/current. |
| `<motor_model>.close_default` | `bool` | Whether the mechanism should initialize toward the closed state by default. |






