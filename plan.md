# Gripper effort semantic update plan

1. Treat gripper-level `GripperCommand.command.max_effort` as force in newtons in the wrapper layer.
2. Add gripper-level defaults:
	- `max_effort`: default closing force in newtons.
	- `max_effort_to_torque_factor`: per-gripper conversion from force to servo torque units.
3. Keep `<motor_model>.control_torque` servo-specific for the low-level `/servo_control` action.
4. Convert wrapper force requests into servo torque before delegating to the Dynamixel action implementation.
5. Report gripper feedback/result effort in newtons while leaving low-level servo feedback/result in torque units.
6. Add an interactive calibration node that steps torque, records externally measured force samples, and estimates `max_effort_to_torque_factor`.
7. Validate with a narrow `colcon build` of the touched packages.
