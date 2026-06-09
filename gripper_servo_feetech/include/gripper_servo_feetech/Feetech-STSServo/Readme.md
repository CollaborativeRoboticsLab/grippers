# Feetech STS Servo — Linux Sweep Example

This repo demonstrates controlling a Feetech STS servo (e.g., STS3215) via the FE-URT-1 interface board from Linux using /dev/ttyUSB0. This is inspired by and replicated the examples in [matthieuvigne/STS_servos](https://github.com/matthieuvigne/STS_servos).

> All of the examples were tested using a [FE-URT1](https://www.makerstore.com.au/product/mb-elc-servo-controller-urt1/) as serial interface.

## What’s Included
- Header-only driver: STSServoDriver.hpp — Linux-friendly version of the original Arduino driver.
- Sweep example: sweep_motion_with_interface_board.cpp — moves the servo to ~0°, ~180°, ~360° repeatedly.

## How It Works
- LinuxSerial wrapper: Minimal serial class that opens /dev/ttyUSB0 using POSIX (open, read, write) and configures the port to 1 Mbps via termios (B1000000).
- Arduino shims: The driver provides small replacements for byte, delay(), delayMicroseconds(), and no-op pinMode() / digitalWrite() so the same protocol logic compiles on Linux.
- Protocol: Packets are built as FF FF | ID | LEN | INSTR | PARAMS | ~CHECKSUM and parsed similarly.
- Servo types: The driver auto-detects STS vs SCS per servo ID and applies the correct endianness and sign handling.

## Build
From the project root, build the examples (header-only driver, no extra sources needed):

```bash
g++ -std=c++17 -O2 -o simple_motion simple_motion.cpp -lpthread
g++ -std=c++17 -O2 -o continous_motion continous_motion.cpp -lpthread
g++ -std=c++17 -O2 -o change_servo_id change_servo_id.cpp -lpthread
g++ -std=c++17 -O2 -o move_synchronously move_synchronously.cpp -lpthread
g++ -std=c++17 -O2 -o velocity_mode velocity_mode.cpp -lpthread
g++ -std=c++17 -O2 -o follow_servo follow_servo.cpp -lpthread
g++ -std=c++17 -O2 -o move_to_home move_to_home.cpp -lpthread
g++ -std=c++17 -O2 -o step_mode step_mode.cpp -lpthread
```

Alternatively, you can use the Makefile for convenience:

```bash
make        # builds all example binaries
make clean  # removes all built binaries
```

## Run
Run the executables (adjust device path and servo ID in the source if needed):

```bash
./simple_motion --port /dev/ttyUSB0 --id 1
./continous_motion --port /dev/ttyUSB0 --id 2
./change_servo_id --port /dev/ttyUSB0 --old 1 --new 2
./move_synchronously --port /dev/ttyUSB0 --id1 1 --id2 2
./velocity_mode --port /dev/ttyUSB0 --id 1
./follow_servo --port /dev/ttyUSB0 --leader 1 --follower 2
./move_to_home --port /dev/ttyUSB0 --id 1
./step_mode --port /dev/ttyUSB0 --id1 1 --id2 2
```

If you see a permission error on /dev/ttyUSB0, either run with sudo or add your user to the dialout group:

```bash
sudo usermod -a -G dialout "$USER"
# Log out/in or reboot for the group change to take effect
```

## Configure
- Serial device: Edit serial_dev in sweep_motion_with_interface_board.cpp and simple_motion.cpp (default: /dev/ttyUSB0).
- Servo ID: Edit SERVO_ID in sweep_motion_with_interface_board.cpp (example uses 2) and simple_motion.cpp (example uses 1).
- Baud rate: The FE-URT-1 board and Feetech STS servos typically use 1 Mbps; the examples set B1000000.

## Motion Sequence
The sweep program:
- Pings the target servo to confirm communication.
- Moves to 0 (≈0°), waits until motion completes.
- Moves to 2048 (≈180°), waits.
- Moves to 4095 (≈360°) at a slower speed, waits.
- Repeats the cycle.

## Files Overview
- STSServoDriver.hpp: Header-only driver, API-compatible with the Arduino version, adapted for Linux.
- linux_serial.hpp: Minimal POSIX serial wrapper providing the `HardwareSerial`-compatible API and a `wait_until_done()` helper.
- sweep_motion_board.cpp: Continuous sweep program (~0° → ~180° → ~360°) for demo boards.
- continous_motion.cpp: Linux sweep example similar to `sweep_motion_board.cpp`.
- simple_motion.cpp: Minimal position sequence (0 → 2048 → 4095 once).
- move_to_home.cpp: Homes one or two servos to position 0 synchronously.
- move_synchronously.cpp: Sends synchronized target positions to two servos.
- velocity_mode.cpp: Runs a single servo in velocity mode with alternating speeds.
- follow_servo.cpp: Makes a follower servo mirror a leader’s position.
- change_servo_id.cpp: Changes a servo’s ID and verifies via ping.
- step_mode.cpp: Drives two servos in STEP mode with synchronous step commands.

## Example Walkthroughs

### simple_motion.cpp
- Purpose: minimal sanity check of position control on a single servo.
- Mode: POSITION via `setMode(0xFE, STSMode::POSITION)` (broadcast).
- Flow: ping → setTargetPosition to 0 → 2048 → 4095 (last with a speed) while polling `isMoving()`.
- Use when: verifying wiring, baud rate, and a single servo responds correctly.
- Run: [./simple_motion](./simple_motion) with `--port` and `--id`.

### continous_motion.cpp
- Purpose: continuous sweep to demonstrate repeatable motion and timing.
- Mode: POSITION; moves 0 → 2048 → 4095 in a loop.
- Flow: ping each cycle, command position, poll `isMoving()`, fixed delays between moves.
- Use when: burn-in, visual demo, or timing analysis on one servo.
- Run: [./continous_motion](./continous_motion) with `--port` and `--id`.

### sweep_motion_board.cpp
- Purpose: board-focused sweep demo (FE-URT-1) mirroring the Arduino-style example.
- Mode: POSITION; similar motion sequence as `continous_motion.cpp`.
- Flow: init → looped positions (0/2048/4095) with movement polling.
- Use when: testing with interface boards; ensures behavior matches hardware docs.
- Run: [./sweep_motion_board](./sweep_motion_board) with `--port` and `--id`.

### move_to_home.cpp
- Purpose: home one or two servos to position 0 synchronously.
- Mode: POSITION; uses `setTargetPositions()` for two IDs.
- Flow: ping both → sync write positions `{0,0}` → wait until both finished.
- Use when: resetting system state or establishing a known mechanical reference.
- Run: [./move_to_home](./move_to_home) with `--id1` and `--id2`.

### move_synchronously.cpp
- Purpose: send the same target positions to two servos in lockstep.
- Mode: POSITION; uses `setTargetPositions()` with shared speed.
- Flow: broadcast mode → two-step sequence (2048 then 4095) → poll until done.
- Use when: coordinated motions, pairing servos for mirrored trajectories.
- Run: [./move_synchronously](./move_synchronously) with `--id1` and `--id2`.

### velocity_mode.cpp
- Purpose: operate a single servo in velocity mode at alternating speeds.
- Mode: VELOCITY via `setMode(SERVO_ID, STSMode::VELOCITY)`.
- Flow: setTargetVelocity(+500) for 4s → setTargetVelocity(-2048) for 2s, loop.
- Use when: testing continuous rotation behavior and speed response.
- Run: [./velocity_mode](./velocity_mode) with `--id`.

### follow_servo.cpp
- Purpose: make one servo mirror another servo’s position.
- Mode: Follower in POSITION; leader torque disabled so you can move it by hand.
- Flow: loop reading leader via `getCurrentPosition()` and writing follower via `setTargetPosition()`.
- Use when: prototyping master-follower linkages or teleoperation demos.
- Run: [./follow_servo](./follow_servo) with `--leader` and `--follower`.

### change_servo_id.cpp
- Purpose: change a servo’s ID safely and verify success.
- Mode: register writes; uses `setId()` which manages write-lock and verification.
- Flow: unlock → write new ID → re-lock → ping new ID with retries → report.
- Use when: organizing bus addresses before multi-servo control.
- Run: [./change_servo_id](./change_servo_id) with `--old` and `--new`.

### step_mode.cpp
- Purpose: drive two servos in STEP mode with synchronized incremental moves.
- Mode: STEP per-ID using `setMode(ID, STSMode::STEP)`.
- Flow: sync-write steps `{+N,+N}` at speed, wait, then `{−N,−N}`; repeat. `--steps` controls N.
- Use when: incremental indexing, jog moves, or testing backlash/repeatability.
- Run: [./step_mode](./step_mode) with `--id1`, `--id2`, and optional `--steps`.

## Notes
- Keep the interface board powered with a suitable supply; USB power alone is often insufficient for servo loads.
- Ensure ground is common between your host and the interface board.
- For multiple servos, use setTargetPositions() to send synchronized commands.
- If you see timeouts, check wiring (TX/RX on FE-URT-1), baud rate (1 Mbps), and that the servo ID matches your device.
