# Servo Parameter reference

## Config selection

| Parameter | Type | Description |
| --- | --- | --- |
| `motor_model` | `string` | Selects the nested preset under `gripper_ros/config/servos/dynamixel.yaml` or `gripper_ros/config/servos/feetech.yaml`, depending on the launched node. |

## Node/runtime behavior

| Parameter | Type | Description |
| --- | --- | --- |
| `shutdown_on_init_failure` | `bool` | Dynamixel-specific. If `true`, the Dynamixel node raises initialization failures instead of staying alive after a startup error. |

Following are the common parameters for most servo configurations. Check the specific YAML files for your motor model for the actual values.


## Servo Transport

| Parameter | Type | Description |
| --- | --- | --- |
| `device_name` | `string` | Serial port, for example `/dev/ttyUSB0`. |
| `baudrate` | `int` | Serial baud rate. |
| `servo_id` | `int` | Motor ID. |

## Servo Position scaling

| Parameter | Type | Description |
| --- | --- | --- |
| `position_is_radians` | `bool` | If `true`, `open_position` and `close_position` are interpreted as radians and converted to ticks internally; if `false`, they are treated as raw ticks directly. |
| `ticks_per_rev` | `int` | Number of ticks per full revolution before gear reduction. |
| `gear_ratio` | `float` | Dynamixel-specific in the current codebase. Gear reduction ratio, for example `1.0` for no reduction or `2.0` for 2:1 reduction. |
| `direction` | `int` | Usually `+1` or `-1`. Multiplier for inverting direction if the motor moves opposite to the expected direction. |
| `zero_offset_ticks` | `int` | Zero position offset in ticks. For example, if the motor's zero position is at 512 ticks, set this to 512 so `open_position: 0.0` corresponds to the physical open position. |

## Servo targets

| Parameter | Type | Description |
| --- | --- | --- |
| `open_position` | `float` | Target open position, in radians by default. |
| `close_position` | `float` | Target close position, in radians by default. |

## Servo motion completion behavior

| Parameter | Type | Description |
| --- | --- | --- |
| `goal_tolerance_ticks` | `int` | Treat the command as successful when `abs(target - present) <= tolerance`. |
| `motion_timeout_sec` | `float` | Abort if the target is not reached in time. |
| `poll_rate_hz` | `float` | Present-position polling frequency. |
| `status_publish_rate_hz` | `float` | Periodic status logging rate. If `0.0`, disable the background status timer. |

## Servo Torque mode

### Common torque-mode parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `use_torque_mode` | `bool` | Default torque-mode behavior if the action goal does not explicitly enable it. |
| `control_torque` | `float` | Default torque request used in torque mode when the action goal leaves `torque` at `0.0`. |
| `safety_torque_limit` | `float` | Position-mode safety threshold; when reached, the node holds the current position instead of reopening. |
| `close_default` | `bool` | Used when the close goal’s `close_ratio` is left at `0.0`; if `true`, the node moves to its configured default close position. |

### Dynamixel-specific

| Parameter | Type | Description |
| --- | --- | --- |
| `torque_per_current_unit` | `float` | Conversion from Dynamixel present-current units to your chosen torque units. Set this if you want action goals and monitoring in Nm. |
| `default_torque` | `float` | Used when goal `torque` is `0.0`. |

### Feetech-specific

| Parameter | Type | Description |
| --- | --- | --- |
| `speed` | `int` | Motion speed parameter used by the Feetech node, for example `4095`. |
| `torque_per_current_unit` | `float` | Conversion from Feetech present-current values to your chosen torque units. The vendored driver currently returns current in amps. |
| `torque_limit_per_torque_unit` | `float` | Conversion from your chosen torque units into the Feetech torque-limit register units. |
| `default_torque_limit` | `float` | Legacy or raw Feetech torque-limit register fallback used when no calibrated torque request is available. |
| `torque_limit_register` | `int` | Register address used for the Feetech torque limit. |

## Servo Communication retry behavior

The low-level Dynamixel driver now retries transient communication failures internally before surfacing an action failure. This is intended to hide brief `There is no status packet!`, CRC, or parse glitches from the external action client when the bus recovers quickly.

| Parameter | Type | Description |
| --- | --- | --- |
| `comm_retry_timeout_sec` | `float` | Maximum retry window for one low-level operation. |
| `comm_retry_initial_delay_sec` | `float` | First retry delay. |
| `comm_retry_max_delay_sec` | `float` | Maximum backoff delay between retries. |
| `comm_retry_backoff` | `float` | Retry backoff multiplier. |
| `comm_retry_reinit_every` | `int` | Re-open the serial port after this many failed attempts. |

Persistent failures still abort the action once the retry window is exhausted.

## Servo Control table addresses (Dynamixel-specific)

These *must* match your motor model’s control table register addresses for the node to work correctly. The current Dynamixel YAML presets are configured for Protocol 2.0 motors like the XM430, but you can adjust these to support other models or protocols.

| Parameter | Type | Description |
| --- | --- | --- |
| `addr_operating_mode` | `int` | For example `11` for Dynamixel Protocol 2.0. |
| `addr_torque_enable` | `int` | For example `64` for Dynamixel Protocol 2.0. |
| `addr_goal_current` | `int` | For example `102` for Dynamixel Protocol 2.0. |
| `addr_goal_position` | `int` | For example `116` for Dynamixel Protocol 2.0. |
| `addr_present_current` | `int` | For example `126` for Dynamixel Protocol 2.0. |
| `addr_present_position` | `int` | For example `132` for Dynamixel Protocol 2.0. |

## Servo Operating mode values (Dynamixel-specific)

| Parameter | Type | Description |
| --- | --- | --- |
| `operating_mode_current` | `int` | Commonly `0`. |
| `operating_mode_position` | `int` | Commonly `3`. |
| `operating_mode_current_based_position` | `int` | Commonly `5`. |


# Torque behavior

- Position mode keeps moving toward the requested open/close target until it reaches the target or the measured torque exceeds `safety_torque_limit`.
- When the safety limit is hit, the node writes the current position back as the hold target, so the gripper stops without reopening.
- Dynamixel torque mode uses `control_torque` unless the action goal supplies a non-zero `torque`, and it stops once the requested torque is reached while keeping the current position.
- Feetech torque mode is an approximation: it applies a torque-limit register value derived from the requested torque and stops once the measured torque estimate reaches the requested threshold.

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

For Feetech STS/SCS servos, the vendored driver currently reports present current in amps. To interpret action `torque`, `control_torque`, and `safety_torque_limit` in physical units such as Nm, you need two empirical calibrations:

- `torque_per_current_unit`: converts measured current (amps) into your chosen torque units
- `torque_limit_per_torque_unit`: converts your chosen torque units into the Feetech torque-limit register units

## Practical note:

- These are first-pass estimates based on stall specs, not precise gripping-force calibration.
- Real gripping torque depends on voltage, temperature, gearbox losses, and your linkage.
- The best way to finalize `torque_per_current_unit` is to start from the datasheet estimate where available, log present current, and calibrate against a measured external torque or fingertip force on your mechanism.
- For Feetech, the resulting behavior is still approximate because the driver uses a torque-limit register rather than true current-based position control.