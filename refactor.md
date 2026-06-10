# Gripper stack refactor plan

## Target package roles

### `gripper_servo_dynamixel`
- Own the **core Dynamixel servo control class**.
- Provide a **simple ROS 2 action node** that exposes `servo_control` for one-servo position and optional torque/current-limited mode.
- Remain the low-level motor package.
- Current implementation language: **Python**.

### `gripper_two_fingers`
- Own the **gripper-level behavior** for the Dynamixel-based two-finger gripper.
- Compose exactly **one `DynamixelServo` object** from `gripper_servo_dynamixel`.
- Expose **`open_gripper`** and **`close_gripper`** actions.
- Publish TF / joint-state-facing gripper articulation state.
- Current implementation language: **Python**.

### `gripper_servo_feetech`
- Own the **core Feetech servo control class / low-level action node**.
- Provide a **simple ROS 2 action node** that exposes `servo_control` for one Feetech servo by position and optional torque-limit mode.
- Remain the low-level motor package.
- Current implementation language: **C++**.

### `gripper_feetech_test`
- Own the **gripper-level Feetech test wrapper**.
- Compose exactly **one Feetech servo object / servo abstraction** from `gripper_servo_feetech`.
- Expose **`open_gripper`** and **`close_gripper`** actions.
- Publish TF / joint-state-facing gripper articulation state.
- Current implementation language: **C++**.

## Behavioral requirements

These behaviors must be consistent across both gripper-level packages:

1. In **`gripper_two_fingers`**, if `open_gripper` is called while the gripper is already open, the action must **succeed**.
2. In **`gripper_two_fingers`**, if `close_gripper` is called while the gripper is already closed, the action must **succeed**.
3. In **`gripper_feetech_test`**, if `open_gripper` is called while the gripper is already open, the action must **succeed**.
4. In **`gripper_feetech_test`**, if `close_gripper` is called while the gripper is already closed, the action must **succeed**.

Interpretation for implementation:
- “Already open/closed” should be determined from servo feedback when available.
- For purely simulated nodes, internal state is acceptable.
- Success in these cases should be explicit in the result message, not treated as an error or timeout.

## Refactor direction

### Phase 1 — align package responsibilities
- [x] Create `gripper_feetech_test` C++ package.
- [x] Ensure launch files point at the correct package/executable names.
- [x] Remove or isolate stale references to legacy package names (`gripper_dynamixel`, `gripper_feetech`).
- [x] Remove redundant legacy code once replacement paths are validated.

### Phase 2 — stabilize low-level servo packages
- [x] `gripper_servo_dynamixel`: keep `servo.py` as the reusable hardware abstraction.
- [x] `gripper_servo_dynamixel`: expose a simple ROS 2 action node for one-servo open/close control.
- [x] `gripper_servo_feetech`: keep low-level servo + action behavior focused on one servo.
- [x] Keep low-level packages free of gripper-specific linkage / TF assumptions.

### Phase 3 — stabilize gripper-level wrappers
- [x] `gripper_two_fingers`: keep only gripper-level semantics, articulation publication, and open/close orchestration.
- [x] `gripper_feetech_test`: keep only gripper-level semantics, articulation publication, and open/close orchestration.
- [x] Add explicit idempotent action handling for already-open / already-closed states.
- [x] Add or refactor TF publication so articulation ownership stays in the gripper-level packages.

### Phase 4 — cleanup
- [x] Remove redundant duplicated parameter declarations and stale code paths.
- [x] Remove old dummy / superseded packages if no launch files, docs, or dependencies still require them.
- [x] Update docs to reflect final package names and responsibilities.

## Implementation notes started in this pass

- `gripper_servo_dynamixel` should no longer be only a status node; it should become a low-level action node.
- `gripper_two_fingers` should check servo feedback before commanding motion so repeated open/close requests succeed immediately.
- `gripper_feetech_test` should maintain simulated gripper state and report immediate success when the requested state is already satisfied.
- `gripper_ros` launch files and package dependencies should reference:
	- `gripper_servo_dynamixel`
	- `gripper_two_fingers`
	- `gripper_servo_feetech`
	- `gripper_feetech_test` only where a gripper-level Feetech test launch is intended.

## Completed in this pass

- Added a real action-server implementation to `gripper_servo_dynamixel`.
- Made `gripper_two_fingers` idempotent for already-open / already-closed requests.
- Made `gripper_feetech_test` idempotent for already-open / already-closed requests.
- Fixed `gripper_ros` launch/package references to the current package names.
- Removed redundant duplicate parameter declarations from `gripper_two_fingers`.

## Completed in the Feetech reuse pass

- Refactored `gripper_feetech_test` into a thin wrapper around `gripper_servo_feetech::FeetechGripperActionNode`.
- Moved Feetech already-open / already-closed success behavior into the reusable low-level node.
- Exported `gripper_servo_feetech` includes/dependencies for downstream reuse.

## Dynamixel reuse pass started

- Refactor `gripper_servo_dynamixel.DynamixelServoActionNode` into the reusable action base for one-servo open/close behavior.
- Refactor `gripper_two_fingers` into a thin subclass that only adds gripper articulation publication.
- Keep joint-state / TF ownership in `gripper_two_fingers`, while action orchestration stays in `gripper_servo_dynamixel`.

## Completed in the Dynamixel articulation ownership pass

- `gripper_two_fingers` now owns articulation publication while reusing the low-level Dynamixel action base.
- Gripper-level launch/config now target `gripper_two_fingers_node` explicitly.
- Motor and gripper overlay parameter files were aligned with the gripper-level wrapper node name.

## Slider-joint articulation pass completed

- Updated the two-finger gripper description to the new exported slider-based model.
- `gripper_two_fingers` now publishes the public slider joints `gripper_planar_4` and `gripper_planar_5` directly.
- `robot_state_publisher` now owns the moving TF tree expansion from those joint states.
- The gripper overlay maps servo `open_position` / `close_position` into per-finger `joint_open_position` / `joint_close_position` values.
- The visualization launch is separate from the hardware/gripper launch so both can run at once without duplicating the node.

## Completed in the action-ownership pass

- `gripper_servo_dynamixel` now exposes low-level `servo_control` instead of low-level `open_gripper` / `close_gripper`.
- `gripper_two_fingers` now owns the gripper-level `open_gripper` / `close_gripper` actions while reusing the same Dynamixel motor logic.
- `gripper_servo_feetech` now exposes low-level `servo_control` instead of low-level `open_gripper` / `close_gripper`.
- `gripper_feetech_test` now owns the gripper-level `open_gripper` / `close_gripper` actions while reusing the same Feetech motor logic.
- Added `gripper_msgs/action/ServoControl.action` as the shared low-level servo command contract.

## Completed in the Feetech articulation ownership pass

- Removed gripper joint-state publication parameters and behavior from `gripper_servo_feetech`.
- Moved Feetech open/close orchestration and articulation publication into `gripper_feetech_test`.
- Added `gripper_ros/config/grippers/feetech_test.yaml` as the gripper-level Feetech overlay.
- Added `gripper_ros/launch/gripper_feetech_test.launch.py` as the gripper-level Feetech launch entrypoint.
- Switched `gripper_ros/config/servos/feetech.yaml` to wildcard `/**` so both low-level and gripper-level Feetech nodes can reuse one motor config.

## Final cleanup status

- No old source package directories remain in `src/grippers`; the active packages are the current package set only.
- The two-finger Dynamixel path now uses only the direct slider-joint articulation model.
- The remaining legacy references were reduced to legitimate node names, URDF link names, and historical notes in this plan.

## Current verified state before hardware tuning

This section is intended as the restart point for the next chat/session.

### What is already working in code

1. `gripper_servo_dynamixel` reads present position from hardware and converts ticks into configured command units.
2. `gripper_two_fingers` receives that scalar and maps it into the two public slider joints:
	- `gripper_planar_4`
	- `gripper_planar_5`
3. The preferred runtime configuration is now:

	```yaml
	gripper_left_finger:
	  joint_name: gripper_planar_4
	  joint_open_position: -0.045
	  joint_close_position: 0.0

	gripper_right_finger:
	  joint_name: gripper_planar_5
	  joint_open_position: -0.045
	  joint_close_position: 0.0
	```

4. `gripper_two_fingers` publishes slider-joint `joint_states`.
5. `robot_state_publisher` expands those into the moving TF tree.
6. The runtime now uses direct per-finger open/close endpoint mapping only; the old array-based compatibility path has been removed.

### Important TF / URDF findings

The current `two-finger-gripper.urdf.xacro` is a slider-based model.
The two public articulation joints are:

- `gripper_planar_4`
- `gripper_planar_5`

The previous helper-joint / support-bridge interpretation is no longer the active runtime model.

### Current recommendation for articulation ownership

- Use `gripper_two_fingers` to publish the two slider joint states.
- Use `robot_state_publisher` for the moving TF tree.
- Keep the hardware/gripper launch and visualization launch separate so they can run together.

### Important caveat about revolute joints

If additional coupled linkage motion is needed beyond what the current slider model provides, that should be solved in the URDF/modeling layer rather than by reintroducing ad hoc TF bridge logic.

### Is position being read and published correctly right now?

Yes, at the current abstraction level.

What is correct:

- the Dynamixel present position is read from hardware
- it is converted into command units via `ticks_to_command_units()`
- the gripper node maps that value into slider positions using per-finger `joint_open_position` / `joint_close_position`
- the resulting `joint_states` drive the simulated finger motion through `robot_state_publisher`

### What was changed now to make the next session easier

- Aligned the code and docs with the direct slider-joint articulation model.
- Removed the stale support-connectivity TF path from the active runtime.
- Removed the old array-based articulation fallback from `gripper_two_fingers`.
- Kept the launch split explicit:
	- `gripper_soft_two_fingers.launch.py` for the independent gripper/hardware node
	- `gripper_sim.launch.py` for visualization only

### Exact next steps for the office computer session

1. Bring up the current stack with hardware attached.
2. Verify that open/close behavior and RViz slider motion still match after restart.
3. Tune `joint_open_position` / `joint_close_position` if the simulated stroke needs adjustment.
4. If higher-fidelity linkage motion is required, revisit the URDF/model rather than adding back legacy fallback logic.

### Suggested question to start the next chat with

Use something like:

> Continue the gripper refactor from `refactor.md`. We are on the slider-joint two-finger model now. The hardware launch and sim launch are split, and `gripper_two_fingers` publishes `gripper_planar_4` / `gripper_planar_5` from servo feedback. Next we want to tune the joint endpoints or simplify the remaining docs and package surfaces.
