# Probe and diagnostics

## Common setup steps / troubleshooting

- **YAML parsing errors**: YAML indentation must use spaces (tabs are invalid YAML). Keep `ros__parameters` blocks consistently indented.
- **Port open failures**: verify `device_name` and permissions (`dialout` group).
- **No response / timeouts**: likely wrong `servo_id`, baudrate, or a control-table address mismatch.
- **Motor moves the wrong way**: set `direction: -1` or adjust your sign convention.
- **Open/close not matching physical endpoints**:
  - Use `zero_offset_ticks` to shift the reference
  - Adjust `open_position` / `close_position`
  - Consider switching to `position_is_radians: false` temporarily and tune in ticks

- If a USB serial adapter is unplugged and replugged, the device path may change from `/dev/ttyUSB0` to `/dev/ttyUSB1` or similar. Recheck `/dev/ttyUSB*` before assuming the motor settings changed.



The repository includes a standalone probe script at `src/grippers/gripper_servo_dynamixel/find_id.py`.

Use it when you need to:

- scan an ID range
- sweep multiple baudrates
- poll common registers from a known servo

### Scan one baudrate across an ID range

```bash
python3 src/grippers/gripper_servo_dynamixel/find_id.py \
  --device /dev/ttyUSB0 \
  --baudrate 57600 \
  --scan-start 0 \
  --scan-end 20 \
  --no-poll
```

### Sweep common baudrates and IDs

```bash
python3 src/grippers/gripper_servo_dynamixel/find_id.py \
  --device /dev/ttyUSB0 \
  --common-baudrate-sweep \
  --scan-start 0 \
  --scan-end 252 \
  --no-poll
```

### Poll a known servo continuously

```bash
python3 src/grippers/gripper_servo_dynamixel/find_id.py \
  --device /dev/ttyUSB0 \
  --id 1 \
  --baudrate 57600 \
  --count 0 \
  --interval 0.5
```

The script prints `model`, `operating_mode`, `torque_enable`, `hardware_error`, `present_current`, and `present_position` when a servo is reachable.

### Interpreting common failures

- `There is no status packet!`: transmit succeeded, but no valid Dynamixel reply came back. Check power, wiring, adapter type, baudrate, protocol version, and ID.

- `Incorrect status packet!`: some bytes came back, but they were not a valid Dynamixel response. This usually points to bus noise, the wrong serial device, or a non-Dynamixel device on that port.

- Port changed from `/dev/ttyUSB0` to `/dev/ttyUSB1`: the USB serial adapter re-enumerated after reconnect/reset. Re-run the probe on the new device path.

- `Permission denied`: fix serial permissions first with the `dialout` group change described above, then log out/in before retrying.