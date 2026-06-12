# Gripper Parameter reference

## Gripper-level common parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `motor_model` | `string` | Used to find the relevant servo config for the underlying motor model. This is used to override specific parameters from servo config. |
| `publish_gripper_joint_states` | `bool` | Whether to publish the gripper joint states on `gripper_joint_state_topic`. If `false`, the node still publishes the underlying servo joint states but not the gripper-level ones. |
| `gripper_joint_state_topic` | `string` | Topic on which to publish the gripper joint states. |
| `gripper_joint_name_prefix` | `string` | Prefix for the gripper joint names. |
| `gripper_joint_state_rate_hz` | `float` | Rate at which to publish the gripper joint states. |






