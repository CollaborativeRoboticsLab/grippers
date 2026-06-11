# Servo Parameter reference

## Config selection

- `motor_model` (string): selects the nested preset under `gripper_ros/config/servos/dynamixel.yaml` or `gripper_ros/config/servos/feetech.yaml`, depending on the launched node.

Following are the common parameters for most servo configurations. Check the specific YAML files for your motor model for the actual values.


## Servo Transport

- `device_name` (string): serial port, e.g. `/dev/ttyUSB0`
- `baudrate` (int): serial baud rate
- `servo_id` (int): motor ID

## Servo Position scaling

- `position_is_radians` (bool) - if true, `open_position` and `close_position` are interpreted as radians and converted to ticks internally; if false, they are treated as raw ticks directly
- `ticks_per_rev` (int) - number of ticks per full revolution (before gear reduction)
- `gear_ratio` (float) - gear reduction ratio (e.g. `1.0` for no reduction, `2.0` for 2:1 reduction)
- `direction` (int, usually `+1` or `-1`) - multiplier for inverting direction if the motor moves opposite to the expected direction
- `zero_offset_ticks` (int) - zero position offset in ticks (e.g. if your motor's zero position is at 512 ticks, set this to 512 so that `open_position: 0.0` corresponds to the physical open position)

## Servo targets

- `open_position` (float): target open position (radians by default)
- `close_position` (float): target close position (radians by default)

## Servo motion completion behavior

- `goal_tolerance_ticks` (int): success when `abs(target - present) <= tolerance`
- `motion_timeout_sec` (float): abort if not reached in time
- `poll_rate_hz` (float): present position polling frequency

## Servo Torque mode

### Dynamixel-specific

- `torque_per_current_unit` (float): conversion from Dynamixel present-current units to your chosen torque units; set this if you want action goals and monitoring in Nm
- `control_torque` (float): default torque request used in torque mode when the action goal leaves `torque` at `0.0`
- `use_torque_mode` (bool): default torque-mode behavior if the action goal does not explicitly enable it
- `default_torque` (float): used when goal `torque` is `0.0`
- `safety_torque_limit` (float): position-mode safety threshold; when reached, the node holds the current position instead of reopening
- `close_default` (bool): used when close goal’s boolean flag is default-constructed / false

### Feetech-specific

- `speed` (int): motion speed parameter used by the Feetech node (e.g. `4095`)
- `default_torque_limit` (float): default Feetech torque-limit register value used when the action goal leaves `torque` at `0.0`
- `torque_limit_register` (int): register address used for the Feetech torque limit

## Servo Communication retry behavior

The low-level Dynamixel driver now retries transient communication failures internally before surfacing an action failure. This is intended to hide brief `There is no status packet!`, CRC, or parse glitches from the external action client when the bus recovers quickly.

- `comm_retry_timeout_sec` (float): maximum retry window for one low-level operation
- `comm_retry_initial_delay_sec` (float): first retry delay
- `comm_retry_max_delay_sec` (float): maximum backoff delay between retries
- `comm_retry_backoff` (float): retry backoff multiplier
- `comm_retry_reinit_every` (int): re-open the serial port after this many failed attempts

Persistent failures still abort the action once the retry window is exhausted.

## Servo Control table addresses (Dynamixel-specific)

These *must* match your motor model’s control table register addresses for the node to work correctly. The current Dynamixel YAML presets are configured for Protocol 2.0 motors like the XM430, but you can adjust these to support other models or protocols.

- `addr_operating_mode` (int): e.g. `11` for Dynamixel Protocol 2.0
- `addr_torque_enable` (int) - e.g. `64` for Dynamixel Protocol 2.0
- `addr_goal_current` (int) - e.g. `102` for Dynamixel Protocol 2.0
- `addr_goal_position` (int) - e.g. `116` for Dynamixel Protocol 2.0
- `addr_present_current` (int) - e.g. `126` for Dynamixel Protocol 2.0
- `addr_present_position` (int) - e.g. `132` for Dynamixel Protocol 2.0

## Servo Operating mode values (Dynamixel-specific)

- `operating_mode_current` (commonly `0`)
- `operating_mode_position` (commonly `3`)
- `operating_mode_current_based_position` (commonly `5`)


# Torque behavior

- Position mode keeps moving toward the requested open/close target until it reaches the target or the measured torque exceeds `safety_torque_limit`.
- When the safety limit is hit, the node writes the current position back as the hold target, so the gripper stops without reopening.
- Torque mode uses `control_torque` unless the action goal supplies a non-zero `torque`, and it stops once the requested torque is reached while keeping the current position.

## Calibrating `torque_per_current_unit`

The Dynamixel control table reports `Goal Current(102)` and `Present Current(126)` in raw current units. For XM430 servos, the ROBOTIS e-Manual lists that unit as about `2.69 mA` per count.

To estimate `torque_per_current_unit` for a specific model, use:

```text
torque_per_current_unit ≈ current_unit_amps * torque_constant_nm_per_amp
```

Where:

```text
current_unit_amps = 0.00269
torque_constant_nm_per_amp ≈ stall_torque_nm / stall_current_amps
```

Examples from the ROBOTIS XM430 pages at 12 V:

- `XM430-W350`: `4.1 Nm / 2.3 A = 1.783 Nm/A`, so `torque_per_current_unit ≈ 0.00269 * 1.783 = 0.00480 Nm/unit`
- `XM430-W210`: `3.0 Nm / 2.3 A = 1.304 Nm/A`, so `torque_per_current_unit ≈ 0.00269 * 1.304 = 0.00351 Nm/unit`

If you are using `XM430-W350`, a reasonable starting YAML value is:

```yaml
torque_per_current_unit: 0.00480
```

## Practical note:

- These are first-pass estimates based on stall specs, not precise gripping-force calibration.
- Real gripping torque depends on voltage, temperature, gearbox losses, and your linkage.
- The best way to finalize `torque_per_current_unit` is to start from the datasheet estimate, log `Present Current`, and calibrate against a measured external torque or fingertip force on your mechanism.