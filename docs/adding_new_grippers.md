# Adding New Grippers

This document describes how to add a new gripper to this repository.

There are three separate pieces to think about:

1. The gripper description: URDF/xacro, meshes, and joint names.
2. The motor configuration: transport and motor-model defaults.
3. The gripper configuration: which motors are used by this gripper and how motor motion maps into gripper joints.

The current repository supports this split for Dynamixel grippers and is prepared to use the same pattern for Feetech grippers later.

## Current Layout

- `gripper_description/`: meshes and xacro macros for gripper geometry.
- `gripper_ros/config/motors/`: motor-model defaults shared by multiple grippers.
- `gripper_ros/config/grippers/`: gripper-specific overlays.
- `gripper_ros/launch/`: launch files that compose motor config and gripper config.

## 1. Add The Description

If the new gripper has new CAD or linkage geometry:

1. Add meshes under `gripper_description/meshes/<gripper-name>/`.
2. Add a xacro macro under `gripper_description/xacro/`.
3. Keep the macro parameterized with:
	- `parent`
	- `prefix`
	- `xyz`
	- `rpy`
4. Prefix every link and joint name with `${prefix}` to avoid collisions.
5. Use stable joint names because the gripper action node may publish `joint_states` for those names.

For example, the current two-finger gripper macro is:

- [`gripper_description/xacro/two-finger-gripper.urdf.xacro`](../gripper_description/xacro/two-finger-gripper.urdf.xacro)

When this macro is included by a larger robot description, the `prefix` argument should usually be the robot TF prefix.

## 2. Add Or Reuse A Motor Config

Motor configs live in:

- [`gripper_ros/config/motors/dynamixel.yaml`](../gripper_ros/config/motors/dynamixel.yaml)
- [`gripper_ros/config/motors/feetech.yaml`](../gripper_ros/config/motors/feetech.yaml)

Add a new motor preset when you need a new actuator family or a new reusable model-specific control table setup.

Use motor config files for parameters such as:

- serial device name
- baud rate
- servo ID / motor ID
- control table addresses
- operating modes
- generic scaling defaults

Use the motor config as a reusable base, not as the final configuration for one concrete gripper.

## 3. Add A Gripper Config Overlay

Gripper configs live in:

- [`gripper_ros/config/grippers/`](../gripper_ros/config/grippers/)

Each file in this directory describes one complete gripper assembly.

For a Dynamixel gripper, the file should usually:

1. Select the motor preset with `motor_model`.
2. Override site-specific motor values if needed.
3. Set `open_position` and `close_position` for this mechanism.
4. Enable and configure gripper joint-state publication if the URDF has moving joints that must appear in TF.

Example:

- [`gripper_ros/config/grippers/soft_two_finger_dynamixel.yaml`](../gripper_ros/config/grippers/soft_two_finger_dynamixel.yaml)

Important fields in a gripper overlay:

- `motor_model`: chooses the preset from the motor YAML.
- `publish_gripper_joint_states`: enables `JointState` publishing from the action node.
- `gripper_joint_state_names`: URDF joint names that should be published.
- `gripper_joint_state_multipliers`: mapping from the measured motor motion to each gripper joint.
- `gripper_joint_state_offsets`: constant offsets for those joints.

For the current single-motor setup, one motor position is mapped into one or more gripper joints with multipliers and offsets.

For a future multi-motor gripper, this part will need to evolve. The likely direction is one of:

1. one action node per motor plus a gripper coordinator node
2. a dedicated multi-motor gripper action node
3. one gripper config that declares multiple motors and a per-joint source mapping

The repository is not fully generalized for multi-motor grippers yet, so start with a single-motor implementation pattern and extend from there.

## 4. Add A Launch File

Launch files live in:

- `gripper_ros/launch/`

For a gripper-level launch, compose:

1. one motor params file from `config/motors/`
2. one gripper params file from `config/grippers/`

The current example is:

- [`gripper_ros/launch/gripper_dynamixel.launch.py`](../gripper_ros/launch/gripper_dynamixel.launch.py)

That launch file loads both parameter files into the action node:

- `motor_params_file`
- `gripper_params_file`

When you add a new gripper for the same motor family, you usually do not need a new launch file if the same node type is used. You only need a new gripper overlay YAML.

When you add a new motor family or a new node type, add a new launch file for that family.

## 5. Connect The Description To The Robot

After the gripper description exists, include it in the parent robot xacro.

For example:

1. include the gripper xacro file
2. instantiate the macro with the correct `parent`
3. pass the robot `tf_prefix` into the gripper `prefix`

This is what keeps the gripper frames unique and aligned with the main robot description.

## 6. Verify TF And Motion

After adding the gripper:

1. launch the action node with the new gripper overlay
2. start `robot_state_publisher` with the full robot description
3. inspect the TF tree
4. send open and close goals
5. confirm the expected moving gripper subtree remains connected

If the TF tree is broken at a moving joint, the most common cause is missing `joint_states` for the mechanism joints.

If the TF tree is connected but the motion is visually wrong, adjust:

- `gripper_joint_state_multipliers`
- `gripper_joint_state_offsets`
- `open_position`
- `close_position`

## 7. Checklist

For a new gripper, the typical checklist is:

1. Add meshes.
2. Add a xacro macro.
3. Keep joint names stable and prefixed.
4. Reuse or add a motor preset.
5. Add a gripper overlay YAML.
6. Reuse or add a launch file.
7. Include the gripper macro in the robot description.
8. Validate TF connectivity and motion.

## Related Docs

- `docs/dynamixel.md`
- `docs/feetech.md`
- `docs/action_interface.md`
