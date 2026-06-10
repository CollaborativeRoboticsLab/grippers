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
- [~] Remove or isolate stale references to legacy package names (`gripper_dynamixel`, `gripper_feetech`).
- [~] Remove redundant legacy code once replacement paths are validated.

### Phase 2 — stabilize low-level servo packages
- [x] `gripper_servo_dynamixel`: keep `servo.py` as the reusable hardware abstraction.
- [x] `gripper_servo_dynamixel`: expose a simple ROS 2 action node for one-servo open/close control.
- [x] `gripper_servo_feetech`: keep low-level servo + action behavior focused on one servo.
- [~] Keep low-level packages free of gripper-specific linkage / TF assumptions.

### Phase 3 — stabilize gripper-level wrappers
- [x] `gripper_two_fingers`: keep only gripper-level semantics, articulation publication, and open/close orchestration.
- [x] `gripper_feetech_test`: keep only gripper-level semantics, articulation publication, and open/close orchestration.
- [x] Add explicit idempotent action handling for already-open / already-closed states.
- [~] Add or refactor TF publication so articulation ownership stays in the gripper-level packages.

### Phase 4 — cleanup
- [ ] Remove redundant duplicated parameter declarations and stale code paths.
- [ ] Remove old dummy / superseded packages if no launch files, docs, or dependencies still require them.
- [ ] Update docs to reflect final package names and responsibilities.

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

## Dynamic support-connectivity TF pass started

- Use `robot_state_publisher` for most of the gripper tree.
- Publish two dynamic TF bridges in `gripper_two_fingers` for the URDF support-connectivity paths.
- Keep the shared Dynamixel motor YAML global via `/**` so low-level and gripper-level nodes reuse one motor-model config tree without duplicate node-name sections.

## Completed in the Dynamixel support-pairing configurability pass

- Moved the support-connectivity parent/child frame pairing, origin, and axis into `gripper_left_finger.*` / `gripper_right_finger.*` parameters.
- Kept the current URDF pairing as the default runtime config:
	- `gripper_planar_4 -> gripper_mgnr09r315hm -> gripper_gripper_support_1`
	- `gripper_planar_5 -> gripper_mgnr09r315hm_1 -> gripper_gripper_support`
- Made it possible to test the opposite support pairing in YAML only by swapping the two `support_child_frame` values.

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

## Current verified state before hardware tuning

This section is intended as the restart point for the next chat/session.

### What is already working in code

1. `gripper_servo_dynamixel` reads the present motor position using the Dynamixel SDK:
	- `DynamixelServo.read_present_position()`
	- then converts ticks to configured command units via `ticks_to_command_units()`.
2. `gripper_two_fingers` receives that converted servo position in its gripper-level wrapper.
3. `gripper_two_fingers` can now use a **simplified per-finger parameter model**:

	```yaml
	gripper_left_finger:
	  joint_name: gripper_planar_4
	  multiplier: 0.0
	  offset: 0.0

	gripper_right_finger:
	  joint_name: gripper_planar_5
	  multiplier: 0.0
	  offset: 0.0
	```

4. The two dynamic support-connectivity TF bridges are now driven from those two per-finger values.
5. The support-connectivity pairing and transform basis are now also configurable per finger:

	```yaml
	gripper_left_finger:
	  support_parent_frame: gripper_mgnr09r315hm
	  support_child_frame: gripper_gripper_support_1
	  support_origin_xyz: [0.129112, -0.00361501, -0.0045]
	  support_axis_xyz: [-1.0, 0.0, 0.0]

	gripper_right_finger:
	  support_parent_frame: gripper_mgnr09r315hm_1
	  support_child_frame: gripper_gripper_support
	  support_origin_xyz: [0.0189804, -0.0025, 0.0045]
	  support_axis_xyz: [1.0, 0.0, 0.0]
	```

6. The old array-based parameter model is still supported in code as a fallback, but the preferred direction is now the `gripper_left_finger` / `gripper_right_finger` model.

### Important TF / URDF findings

From `gripper_description/xacro/two-finger-gripper.urdf.xacro`, the current URDF connectivity is:

- `gripper_mgnr09r315hm -> gripper_planar_4 -> ... -> gripper_gripper_support_1`
- `gripper_mgnr09r315hm_1 -> gripper_planar_5 -> ... -> gripper_gripper_support`

So the current xacro pairs are:

- `gripper_planar_4` corresponds to `gripper_mgnr09r315hm -> gripper_gripper_support_1`
- `gripper_planar_5` corresponds to `gripper_mgnr09r315hm_1 -> gripper_gripper_support`

This is different from the earlier assumption that the support names matched the same-side rail names directly.
If the mechanism should instead be:

- `gripper_mgnr09r315hm -> gripper_gripper_support`
- `gripper_mgnr09r315hm_1 -> gripper_gripper_support_1`

then the xacro/URDF needs to be reviewed and possibly corrected.

### Current recommendation for articulation ownership

- Use `robot_state_publisher` for the bulk of the tree.
- Use `gripper_two_fingers` only for the two dynamic support-connectivity transforms.
- Do **not** publish `gripper_revolute_*` from the node for now.

### Important caveat about revolute joints

Right now the request is to avoid publishing `gripper_revolute_*` because they are expected to “move on their own” when `planar_4` / `planar_5` move.

In plain URDF + `robot_state_publisher`, that coupling does **not** happen automatically unless:

- the URDF uses explicit `mimic` relationships, or
- another node publishes those joint states, or
- the mechanism is re-modeled so those links are connected by the direct TF bridges instead of independent joints.

So, for the next session, remember:

- if `gripper_revolute_*` are not published, they will remain at their default/zero state in `robot_state_publisher`
- this may be acceptable if only the support connectivity is needed
- it is **not** enough if the full coupled linkage motion must be visualized correctly

### Current recommendation on planar joints

For now, use only:

- `gripper_planar_4`
- `gripper_planar_5`

and treat:

- `gripper_planar_4_1`
- `gripper_planar_4_2`
- `gripper_planar_5_1`
- `gripper_planar_5_2`

as ignored / fixed-at-zero unless hardware testing proves that additional lateral translation or yaw is actually required.

This is now reflected in the preferred config model.

### How the finger displacement should be computed

Because there is one motor and the fingers are mechanically coupled, the intended model is:

- let `distance` = total separation change between the two finger supports
- then each finger support displacement should be approximately `distance / 2`

So the per-finger joint values should come from the measured/estimated gripper opening relative to center, not from independent left/right sensors.

In config form, the current implementation expects:

- `gripper_left_finger.multiplier`
- `gripper_left_finger.offset`
- `gripper_right_finger.multiplier`
- `gripper_right_finger.offset`

to map **servo position** into **left/right support displacement**.

### Is position being read and published correctly right now?

**Partially yes.**

What is correct:

- the Dynamixel present position is read from hardware through `read_present_position()`
- that reading is converted into command units with `ticks_to_command_units()`
- the resulting scalar is then mapped into per-finger values via `multiplier` and `offset`

What is still incomplete:

- the current left/right finger multipliers are still `0.0`
- therefore the support TF bridges currently publish valid frame IDs/origins, but no meaningful motion yet
- this must be tuned with real hardware or measured mechanism geometry
- the URDF-default support pairing is now configurable, but still needs hardware confirmation

### What was changed now to make the next session easier

- Added a simplified per-finger parameter model under:
  - `gripper_left_finger.*`
  - `gripper_right_finger.*`
- Updated the preferred gripper overlay config to use that structure.
- Removed the need to publish `gripper_revolute_*` from the current overlay.
- Stopped relying on `planar_4_*` / `planar_5_*` secondary joints for the dynamic support bridges.
- Kept the older array-based parameter path as fallback compatibility in code.

### Exact next steps for the office computer session

1. Bring up the current stack and inspect TF with the real gripper attached.
2. Verify whether the current URDF side pairing is physically correct:
	- `mgnr09r315hm -> support_1`
	- `mgnr09r315hm_1 -> support`
   - if not correct, swap only:
		- `gripper_left_finger.support_child_frame`
		- `gripper_right_finger.support_child_frame`
3. Measure or estimate the real relation between servo position and finger spacing.
4. Set nonzero values for:

	```yaml
	gripper_left_finger:
	  joint_name: gripper_planar_4
	  multiplier: ???
	  offset: ???

	gripper_right_finger:
	  joint_name: gripper_planar_5
	  multiplier: ???
	  offset: ???
	```

5. Start with symmetric values of equal magnitude, opposite sign if the supports move in opposite x directions.
6. Check whether `planar_4_1/4_2` and `planar_5_1/5_2` truly stay negligible.
7. If not negligible, decide one of:
	- reintroduce secondary planar terms in the wrapper node
	- remodel the URDF to better represent the mechanism
8. If full linkage visualization is required, revisit how `gripper_revolute_*` should be handled:
	- publish them explicitly from kinematic logic, or
	- add `mimic` / revised modeling if possible.

### Suggested question to start the next chat with

Use something like:

> Continue the gripper refactor from `refactor.md`. We are at the Dynamixel two-finger support-connectivity tuning stage. The code already reads motor position correctly and maps it through `gripper_left_finger` / `gripper_right_finger` parameters, but the multipliers are still zero and the URDF support pairing needs verification against hardware.
