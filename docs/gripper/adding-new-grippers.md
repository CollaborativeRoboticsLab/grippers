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

If you need to simulate the new gripper alone without rest of the robot, add/use a simulation launch file that includes the gripper xacro and starts `robot_state_publisher` with the resulting URDF.

- [`gripper_ros/launch/gripper_sim.launch.py`](../gripper_ros/launch/gripper_sim.launch.py)
- [`gripper_description/xacro/two-finger-gripper-standalone.urdf.xacro`](../gripper_description/xacro/two-finger-gripper-standalone.urdf.xacro)

## 2. Add Or Reuse A Motor Config

Motor configs live in:

- [`gripper_ros/config/servos/dynamixel.yaml`](../gripper_ros/config/servos/dynamixel.yaml)
- [`gripper_ros/config/servos/feetech.yaml`](../gripper_ros/config/servos/feetech.yaml)

Add a new motor preset when you need a new actuator family or a new reusable model-specific control table setup.

Use motor config files for parameters such as:

- serial device name
- baud rate
- servo ID / motor ID
- control table addresses
- operating modes
- generic scaling defaults

Use the motor config as a reusable base, not as the final configuration for one concrete gripper.

## 3. Add A Gripper Config Overlay and Action Interface

Gripper configs live in:

- [`gripper_ros/config/grippers/`](../gripper_ros/config/grippers/)

Each file in this directory describes one complete gripper assembly.

Important: the top-level YAML key must match the launched ROS node name. For example, the current two-finger wrapper uses `gripper_two_fingers_node`, while the low-level Dynamixel-only node uses `gripper_dynamixel_action_node`.

Current action ownership pattern:

1. low-level servo packages expose `/servo_control`
2. gripper-level wrapper packages expose `/open_gripper` and `/close_gripper`
3. gripper overlays should describe gripper semantics and articulation, not standalone low-level servo debugging APIs

For a Dynamixel gripper overlay, the file should usually:

1. Select the motor preset with `motor_model`.
2. Override site-specific motor values if needed.
3. Set `open_position` and `close_position` for this mechanism.
4. Enable and configure gripper joint-state publication if the URDF has moving joints that must appear in TF.

Example:

- [`gripper_ros/config/grippers/soft_two_finger_dynamixel.yaml`](../gripper_ros/config/grippers/soft_two_finger_dynamixel.yaml)

Important fields in a gripper overlay:

- `motor_model`: chooses the preset from the motor YAML.
- `publish_gripper_joint_states`: enables `JointState` publishing from the action node.

Preferred current articulation model for the two-finger gripper:

- `gripper_left_finger.joint_name`
- `gripper_left_finger.joint_open_position`
- `gripper_left_finger.joint_close_position`
- `gripper_right_finger.joint_name`
- `gripper_right_finger.joint_open_position`
- `gripper_right_finger.joint_close_position`

For the current single-motor setup, one motor position is mapped into one or more gripper joints.
For the current two-finger slider model, the preferred path is direct open/close endpoint mapping rather than raw multipliers.

For the current coupled two-finger mechanism, the preferred interpretation is:

- left finger displacement from center
- right finger displacement from center
- each side approximately equal to half of the total finger spacing change

For the current preferred setup, use only the two public slider joints:

- `gripper_planar_4`
- `gripper_planar_5`

For a future multi-motor gripper, this part will need to evolve. The likely direction is one of:

1. one action node per motor plus a gripper coordinator node
2. a dedicated multi-motor gripper action node
3. one gripper config that declares multiple motors and a per-joint source mapping

The repository is not fully generalized for multi-motor grippers yet, so start with a single-motor implementation pattern and extend from there.

## 4. Add A Launch File

Launch files live in:

- `gripper_ros/launch/`

For a gripper-level launch, compose:

1. one motor params file from `config/servos/`
2. one gripper params file from `config/grippers/`

The current example is:

- [`gripper_ros/launch/dynamixel.launch.py`](../gripper_ros/launch/dynamixel.launch.py)

The current gripper-level Dynamixel wrapper launch is:

- [`gripper_ros/launch/gripper_soft_two_fingers.launch.py`](../gripper_ros/launch/gripper_soft_two_fingers.launch.py)

That launch file loads both parameter files into the gripper-level node:

- `motor_params_file`
- `gripper_params_file`

When you add a new gripper for the same motor family, you usually do not need a new launch file if the same gripper-level node type is used. You only need a new gripper overlay YAML.

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

For the current two-finger Dynamixel wrapper, the intended architecture is:

- `gripper_two_fingers` publishing the slider `joint_states`
- `robot_state_publisher` expanding those into the moving TF tree

Do not assume that omitted revolute joints will move automatically unless the URDF explicitly models that coupling.

If the TF tree is connected but the motion is visually wrong, adjust:

- `open_position`
- `close_position`
- `gripper_left_finger.joint_open_position`
- `gripper_left_finger.joint_close_position`
- `gripper_right_finger.joint_open_position`
- `gripper_right_finger.joint_close_position`

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

## Adding A New Gripper To The Manipulator

If you want to add this gripper to a manipulator, you can follow the instructions in the [Adding New Manipulator Components](https://github.com/CollaborativeRoboticsLab/grasping/blob/main/docs/manipulator/adding_new_components.md) document.

## Related Docs

- `docs/servo/parameters.md`
- `docs/servo/ros2-interface.md`
- `docs/gripper/parameters.md`
- `docs/gripper/ros2-interface.md`
