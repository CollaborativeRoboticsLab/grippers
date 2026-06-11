# Gripper Parameter reference

## Gripper-level common parameters

- `motor_model` (string): used to find the relavent servo config for the underlying motor model; this is used to override specific parameters from servo config.

- `publish_gripper_joint_states` (bool) - whether to publish the gripper joint states on `gripper_joint_state_topic` (if false, the node will still publish the underlying servo joint states but not the gripper-level ones)
- `gripper_joint_state_topic` (string) - the topic on which to publish the gripper joint states
- `gripper_joint_name_prefix` (string) - the prefix for the gripper joint names
- `gripper_joint_state_rate_hz` (float) - the rate at which to publish the gripper joint states






