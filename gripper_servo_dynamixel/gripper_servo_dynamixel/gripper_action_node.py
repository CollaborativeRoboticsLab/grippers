"""Low-level ROS 2 action node for a single Dynamixel servo.

This node stays intentionally below gripper-level linkage logic. It owns one
``DynamixelServo`` object and exposes:

- ``servo_control`` for direct low-level position commands.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from gripper_msgs.action import ServoControl
from gripper_servo_dynamixel.servo import DynamixelServo, DynamixelServoConfig


@dataclass(frozen=True)
class MotionLoopResult:
    success: bool
    reason: str


class DynamixelServoActionNode(Node):
    def __init__(
        self,
        *,
        node_name: str = 'gripper_dynamixel_action_node',
        default_status_publish_rate_hz: float = 1.0,
        enable_ros_interface: bool = True,
    ) -> None:
        super().__init__(node_name)

        self.declare_parameter('shutdown_on_init_failure', True)
        self.declare_parameter('motor_model', 'XM430')
        self.motor_model = str(self.get_parameter('motor_model').value)

        self._servo_config = DynamixelServoConfig.from_node(self, self.motor_model)
        self._servo: Optional[DynamixelServo] = None
        self._servo_control_server: Optional[ActionServer] = None

        self.declare_parameter('status_publish_rate_hz', float(default_status_publish_rate_hz))

        if enable_ros_interface:
            self._servo_control_server = ActionServer(
                self,
                ServoControl,
                'servo_control',
                execute_callback=self._execute_servo_control,
                goal_callback=self._goal_cb,
                cancel_callback=self._cancel_cb,
            )

        try:
            self._servo = DynamixelServo(self._servo_config)
            self.get_logger().info(
                f"Loaded Dynamixel servo config for model={self.motor_model}, id={self._servo_config.servo_id}, device={self._servo_config.device_name}"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to initialize Dynamixel servo: {exc}')
            if bool(self.get_parameter('shutdown_on_init_failure').value):
                raise

        status_rate = float(self.get_parameter('status_publish_rate_hz').value)
        if status_rate > 0.0:
            self.create_timer(1.0 / status_rate, self._publish_status)

        self.get_logger().info('Dynamixel servo action node ready.')

    def _read_present_position(self) -> int:
        if self._servo is None:
            raise RuntimeError('Servo is not initialized.')
        return self._servo.read_present_position()

    def _handle_position_feedback(self, current_ticks: int) -> None:
        """Hook for subclasses that want joint-state or TF side effects."""

    def _handle_torque_feedback(self, current_torque: float) -> None:
        """Hook for subclasses that want torque-related side effects."""

    def _handle_servo_unavailable_feedback(self) -> None:
        """Hook for subclasses that want fallback publication when servo is absent."""

    def _goal_cb(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _resolve_torque(self, requested_torque: float) -> float:
        if abs(requested_torque) > 0.0:
            return float(requested_torque)
        if abs(self._servo_config.control_torque) > 0.0:
            return float(self._servo_config.control_torque)
        return float(self._servo_config.default_torque)

    def _resolve_use_torque_mode(self, goal_flag: bool) -> bool:
        return bool(goal_flag) or bool(self._servo_config.use_torque_mode)

    def _resolve_servo_control_use_torque_mode(self, requested_torque: float) -> bool:
        return self._resolve_use_torque_mode(abs(float(requested_torque)) > 0.0)

    def _is_at_target(self, target_ticks: int) -> bool:
        if self._servo is None:
            return False

        tolerance = int(self._servo_config.goal_tolerance_ticks)
        current_ticks = self._read_present_position()
        self._handle_position_feedback(current_ticks)
        return abs(int(target_ticks) - int(current_ticks)) <= tolerance

    def _read_present_torque(self) -> float:
        if self._servo is None:
            raise RuntimeError('Servo is not initialized.')
        current_torque = self._servo.read_present_torque()
        self._handle_torque_feedback(current_torque)
        return current_torque

    def _hold_current_position(self, current_ticks: Optional[int] = None) -> Optional[int]:
        if self._servo is None:
            return None

        hold_ticks = self._read_present_position() if current_ticks is None else int(current_ticks)
        self._servo.write_goal_position(hold_ticks)
        return hold_ticks

    def _apply_position(self, target_position: float) -> int:
        if self._servo is None:
            raise RuntimeError('Servo is not initialized.')

        target_ticks = self._servo.position_to_ticks(target_position)
        self._servo.disable_torque()
        self._servo.set_operating_mode(self._servo._driver.mode_position)
        self._servo.enable_torque()
        self._servo.write_goal_position(target_ticks)
        return target_ticks

    def _apply_torque(self, torque: float, target_position: Optional[float]) -> Optional[int]:
        if self._servo is None:
            raise RuntimeError('Servo is not initialized.')

        goal_current = self._servo.torque_to_current_raw(torque)
        self._servo.disable_torque()
        if target_position is None:
            self._servo.set_operating_mode(self._servo._driver.mode_current)
            self._servo.enable_torque()
            self._servo.write_goal_current(goal_current)
            return None

        target_ticks = self._servo.position_to_ticks(float(target_position))
        self._servo.set_operating_mode(self._servo._driver.mode_current_based_position)
        self._servo.enable_torque()
        self._servo.write_goal_current(goal_current)
        self._servo.write_goal_position(target_ticks)
        return target_ticks

    def _run_motion_loop(
        self,
        goal_handle,
        feedback_cls,
        *,
        target_ticks: Optional[int],
        use_torque_mode: bool,
        target_torque: float,
        safety_torque_limit: float,
    ) -> MotionLoopResult:
        timeout = float(self._servo_config.motion_timeout_sec)
        poll_rate = float(self._servo_config.poll_rate_hz)
        sleep_sec = 1.0 / poll_rate if poll_rate > 0.0 else 1.0
        tolerance = int(self._servo_config.goal_tolerance_ticks)

        start_time = time.monotonic()
        start_pos: Optional[int] = None
        if self._servo is not None:
            start_pos = self._read_present_position()

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MotionLoopResult(success=False, reason='canceled')

            elapsed = time.monotonic() - start_time
            if elapsed > timeout:
                return MotionLoopResult(success=False, reason='timeout')

            progress = 0.0
            if self._servo is not None and target_ticks is not None:
                pos = self._read_present_position()
                self._handle_position_feedback(pos)
                applied_torque = self._read_present_torque()
                err = abs(int(target_ticks) - int(pos))
                if err <= tolerance:
                    goal_handle.publish_feedback(feedback_cls(progress=1.0))
                    return MotionLoopResult(success=True, reason='position_reached')

                if use_torque_mode and abs(target_torque) > 0.0 and abs(applied_torque) >= abs(target_torque):
                    self._hold_current_position(pos)
                    goal_handle.publish_feedback(feedback_cls(progress=1.0))
                    return MotionLoopResult(success=True, reason='target_torque_reached')

                if safety_torque_limit > 0.0 and abs(applied_torque) >= abs(safety_torque_limit):
                    self._hold_current_position(pos)
                    goal_handle.publish_feedback(feedback_cls(progress=1.0))
                    return MotionLoopResult(success=True, reason='safety_torque_limit_reached')

                if start_pos is not None:
                    start_err = max(1, abs(int(target_ticks) - int(start_pos)))
                    progress = float(max(0.0, min(1.0, 1.0 - (err / float(start_err)))))
            else:
                progress = float(max(0.0, min(1.0, elapsed / timeout)))
                self._handle_servo_unavailable_feedback()

            goal_handle.publish_feedback(feedback_cls(progress=float(progress)))
            time.sleep(sleep_sec)

    def _execute_position_goal(
        self,
        goal_handle,
        feedback_cls,
        result_cls,
        *,
        target_position: float,
        torque: float,
        use_torque_mode: bool,
        already_message: str,
        success_message: str,
        torque_reached_message: str,
        safety_limit_message: str,
        timeout_message: str,
        canceled_message: str,
        failure_prefix: str,
    ):
        try:
            requested_ticks = self._servo.position_to_ticks(target_position) if self._servo is not None else None
            if requested_ticks is not None and self._is_at_target(requested_ticks):
                goal_handle.succeed()
                return result_cls(success=True, message=already_message)

            target_ticks = self._apply_torque(torque, target_position) if use_torque_mode else self._apply_position(target_position)
            motion_result = self._run_motion_loop(
                goal_handle,
                feedback_cls,
                target_ticks=target_ticks,
                use_torque_mode=use_torque_mode,
                target_torque=torque,
                safety_torque_limit=float(self._servo_config.safety_torque_limit),
            )
            if not motion_result.success:
                if motion_result.reason == 'canceled':
                    return result_cls(success=False, message=canceled_message)
                goal_handle.abort()
                return result_cls(success=False, message=timeout_message)

            if motion_result.reason == 'target_torque_reached':
                goal_handle.succeed()
                return result_cls(success=True, message=torque_reached_message)

            if motion_result.reason == 'safety_torque_limit_reached':
                goal_handle.succeed()
                return result_cls(success=True, message=safety_limit_message)

            goal_handle.succeed()
            return result_cls(success=True, message=success_message)
        except Exception as exc:  # noqa: BLE001
            goal_handle.abort()
            return result_cls(success=False, message=f'{failure_prefix}: {exc}')

    def _execute_servo_control(self, goal_handle) -> ServoControl.Result:
        goal = goal_handle.request
        position = float(goal.position)
        torque = self._resolve_torque(float(goal.torque))
        use_torque_mode = self._resolve_servo_control_use_torque_mode(float(goal.torque))
        return self._execute_position_goal(
            goal_handle,
            ServoControl.Feedback,
            ServoControl.Result,
            target_position=position,
            torque=torque,
            use_torque_mode=use_torque_mode,
            already_message='Servo already at requested position.',
            success_message='Servo command sent.',
            torque_reached_message='Servo reached the requested torque and is holding position.',
            safety_limit_message='Servo reached the safety torque limit and is holding position.',
            timeout_message='Servo command timed out or was canceled.',
            canceled_message='Servo command was canceled.',
            failure_prefix='Servo control failed',
        )

    def _publish_status(self) -> None:
        if self._servo is None:
            self.get_logger().warn('Dynamixel servo is not initialized.')
            return

        try:
            ticks = self._read_present_position()
            position = self._servo.ticks_to_command_units(ticks)
            torque = self._read_present_torque()
            self.get_logger().info(f'Present position: {position} ({ticks} ticks), present torque: {torque:.3f}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Unable to read present position: {exc}')

    def destroy_node(self) -> bool:
        if self._servo_control_server is not None:
            self._servo_control_server.destroy()
        if self._servo is not None:
            self._servo.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[DynamixelServoActionNode] = None
    try:
        node = DynamixelServoActionNode()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('gripper_dynamixel_action_node').fatal(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
