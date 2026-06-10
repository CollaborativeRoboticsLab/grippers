from __future__ import annotations
import math
from typing import Optional

import rclpy
from gripper_msgs.action import CloseGripper, OpenGripper
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState

from gripper_servo_dynamixel.gripper_action_node import DynamixelServoActionNode


class GripperTwoFingersNode(DynamixelServoActionNode):
    def __init__(self) -> None:
        super().__init__(
            node_name='gripper_two_fingers_node',
            default_status_publish_rate_hz=0.0,
            enable_ros_interface=False,
        )

        self._open_server = ActionServer(
            self,
            OpenGripper,
            'open_gripper',
            execute_callback=self._execute_open,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self._close_server = ActionServer(
            self,
            CloseGripper,
            'close_gripper',
            execute_callback=self._execute_close,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self.declare_parameter('publish_gripper_joint_states', False)
        self.declare_parameter('gripper_joint_state_topic', 'joint_states')
        self.declare_parameter('gripper_joint_name_prefix', '')
        self.declare_parameter('gripper_left_finger.joint_name', '')
        self.declare_parameter('gripper_left_finger.joint_open_position', float('nan'))
        self.declare_parameter('gripper_left_finger.joint_close_position', float('nan'))
        self.declare_parameter('gripper_right_finger.joint_name', '')
        self.declare_parameter('gripper_right_finger.joint_open_position', float('nan'))
        self.declare_parameter('gripper_right_finger.joint_close_position', float('nan'))
        self.declare_parameter('gripper_joint_state_rate_hz', 10.0)

        self._joint_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter('gripper_joint_state_topic').value),
            10,
        )
        self._joint_state_timer = None

        if bool(self.get_parameter('publish_gripper_joint_states').value):
            rate_hz = float(self.get_parameter('gripper_joint_state_rate_hz').value)
            if rate_hz > 0.0:
                self._joint_state_timer = self.create_timer(1.0 / rate_hz, self._publish_gripper_joint_states_timer_cb)
            else:
                self._publish_gripper_joint_states_from_feedback()

        self.get_logger().info('Two-finger gripper node ready.')

    def _prefixed_joint_name(self, base_name: str) -> str:
        prefix = str(self.get_parameter('gripper_joint_name_prefix').value)
        if not prefix or base_name.startswith(prefix):
            return base_name
        return prefix + base_name

    def _finger_configs(self) -> list[dict[str, float | str]]:
        configs: list[dict[str, float | str]] = []
        for side in ('left', 'right'):
            base = f'gripper_{side}_finger'
            joint_name = str(self.get_parameter(f'{base}.joint_name').value)
            if not joint_name:
                continue

            configs.append(
                {
                    'side': side,
                    'joint_name': self._prefixed_joint_name(joint_name),
                    'joint_open_position': float(self.get_parameter(f'{base}.joint_open_position').value),
                    'joint_close_position': float(self.get_parameter(f'{base}.joint_close_position').value),
                }
            )

        return configs

    def _finger_joint_position(self, config: dict[str, float | str], command_position_value: Optional[float]) -> float:
        scalar = 0.0 if command_position_value is None else float(command_position_value)

        joint_open = float(config['joint_open_position'])
        joint_close = float(config['joint_close_position'])
        if math.isnan(joint_open) or math.isnan(joint_close):
            joint_name = str(config['joint_name'])
            raise RuntimeError(
                f'{joint_name} requires both joint_open_position and joint_close_position to be configured.'
            )

        command_open = float(self._servo_config.open_position)
        command_close = float(self._servo_config.close_position)
        if math.isclose(command_close, command_open):
            return joint_open

        alpha = (scalar - command_open) / (command_close - command_open)
        alpha = max(0.0, min(1.0, alpha))
        return joint_open + (alpha * (joint_close - joint_open))

    def _joint_state_entries(self, command_position_value: Optional[float]) -> list[tuple[str, float]]:
        finger_configs = self._finger_configs()
        entries: list[tuple[str, float]] = []
        for config in finger_configs:
            joint_name = str(config['joint_name'])
            entries.append(
                (
                    joint_name,
                    self._finger_joint_position(config, command_position_value),
                )
            )
        return entries

    def _publish_gripper_joint_states(self, command_position_value: Optional[float]) -> None:
        if not bool(self.get_parameter('publish_gripper_joint_states').value):
            return

        try:
            entries = self._joint_state_entries(command_position_value)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return

        if not entries:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [name for name, _ in entries]
        msg.position = [position for _, position in entries]
        self._joint_state_pub.publish(msg)

    def _publish_gripper_joint_states_from_feedback(self) -> None:
        if not bool(self.get_parameter('publish_gripper_joint_states').value):
            return

        if self._servo is None:
            self._publish_gripper_joint_states(command_position_value=0.0)
            return

        try:
            current_ticks = self._read_present_position()
            command_units = self._servo.ticks_to_command_units(current_ticks)
            self._publish_gripper_joint_states(command_units)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f'Failed to publish gripper joint states from feedback: {type(exc).__name__}: {exc}'
            )

    def _publish_gripper_joint_states_timer_cb(self) -> None:
        self._publish_gripper_joint_states_from_feedback()

    def _handle_position_feedback(self, current_ticks: int) -> None:
        if self._servo is None:
            return
        command_units = self._servo.ticks_to_command_units(current_ticks)
        self._publish_gripper_joint_states(command_units)

    def _handle_servo_unavailable_feedback(self) -> None:
        self._publish_gripper_joint_states(command_position_value=0.0)

    def _log_gripper_action_request(self, action_name: str, *, torque: float, use_torque_mode: bool) -> None:
        self.get_logger().info(
            f'{action_name} requested (torque={torque:.3f}, use_torque_mode={use_torque_mode})'
        )

    def _log_gripper_action_result(self, action_name: str, result_message: str, *, success: bool) -> None:
        log = self.get_logger().info if success else self.get_logger().warning
        log(f'{action_name} result: {result_message}')

    def _execute_open(self, goal_handle) -> OpenGripper.Result:
        goal = goal_handle.request
        torque = self._resolve_torque(float(goal.torque))
        use_torque_mode = self._resolve_use_torque_mode(bool(goal.use_torque_mode))
        self._log_gripper_action_request('open_gripper', torque=torque, use_torque_mode=use_torque_mode)
        result = self._execute_position_goal(
            goal_handle,
            OpenGripper.Feedback,
            OpenGripper.Result,
            target_position=float(self._servo_config.open_position),
            torque=torque,
            use_torque_mode=use_torque_mode,
            already_message='Servo already at open position.',
            success_message='Open command sent.',
            timeout_message='Open timed out or was canceled.',
            failure_prefix='Open failed',
        )
        self._log_gripper_action_result('open_gripper', result.message, success=bool(result.success))
        return result

    def _execute_close(self, goal_handle) -> CloseGripper.Result:
        goal = goal_handle.request
        close_requested = bool(goal.close) or bool(self._servo_config.close_default)
        torque = self._resolve_torque(float(goal.torque))
        use_torque_mode = self._resolve_use_torque_mode(bool(goal.use_torque_mode))
        self._log_gripper_action_request('close_gripper', torque=torque, use_torque_mode=use_torque_mode)
        if not close_requested:
            goal_handle.succeed()
            result = CloseGripper.Result(success=True, message='Close goal flag was false; no action taken.')
            self._log_gripper_action_result('close_gripper', result.message, success=True)
            return result

        result = self._execute_position_goal(
            goal_handle,
            CloseGripper.Feedback,
            CloseGripper.Result,
            target_position=float(self._servo_config.close_position),
            torque=torque,
            use_torque_mode=use_torque_mode,
            already_message='Servo already at close position.',
            success_message='Close command sent.',
            timeout_message='Close timed out or was canceled.',
            failure_prefix='Close failed',
        )
        self._log_gripper_action_result('close_gripper', result.message, success=bool(result.success))
        return result

    def destroy_node(self) -> bool:
        self._open_server.destroy()
        self._close_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[GripperTwoFingersNode] = None
    try:
        node = GripperTwoFingersNode()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('gripper_two_fingers_node').fatal(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
