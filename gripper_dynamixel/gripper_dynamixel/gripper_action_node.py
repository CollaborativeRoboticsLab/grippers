"""Action servers for opening/closing a Dynamixel-based custom gripper.

This node exposes two actions (defined in gripper_msgs):
- OpenGripper
- CloseGripper

The node uses ROS parameters for the open/close target angles:
- open_position
- close_position

It supports two modes:
- Position mode: command the target position.
- Torque mode: command a torque/current value (application-specific).

Note: This implementation provides the ROS 2 action interface and parameter handling.
Hardware-specific Dynamixel control should be added in `_apply_position` / `_apply_torque`.
"""

from __future__ import annotations

import math
import os
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.parameter import Parameter

from gripper_msgs.action import CloseGripper, OpenGripper
from dynamixel_sdk import PacketHandler, PortHandler

class DynamixelProtocol2Driver:
    def __init__(
        self,
        *,
        device_name: str,
        baudrate: int,
        dxl_id: int,
        addr_operating_mode: int,
        addr_torque_enable: int,
        addr_goal_current: int,
        addr_goal_position: int,
        addr_present_current: int,
        addr_present_position: int,
        operating_mode_current: int,
        operating_mode_position: int,
        operating_mode_current_based_position: int,
    ) -> None:
        if PortHandler is None or PacketHandler is None:
            raise RuntimeError(
                'dynamixel_sdk is not available. Install it (pip: dynamixel-sdk or apt: python3-dynamixel-sdk)'
            )

        self._dxl_id = int(dxl_id)
        self._port = PortHandler(device_name)
        self._packet = PacketHandler(2.0)
        self._device_name = device_name
        self._baudrate = int(baudrate)

        self.addr_operating_mode = int(addr_operating_mode)
        self.addr_torque_enable = int(addr_torque_enable)
        self.addr_goal_current = int(addr_goal_current)
        self.addr_goal_position = int(addr_goal_position)
        self.addr_present_current = int(addr_present_current)
        self.addr_present_position = int(addr_present_position)

        self.mode_current = int(operating_mode_current)
        self.mode_position = int(operating_mode_position)
        self.mode_current_based_position = int(operating_mode_current_based_position)

        self._open()

    def _open(self) -> None:
        if not self._port.openPort():
            raise RuntimeError(f'Failed to open Dynamixel port: {self._device_name}')
        if not self._port.setBaudRate(self._baudrate):
            raise RuntimeError(f'Failed to set baudrate={self._baudrate} on {self._device_name}')

    def close(self) -> None:
        try:
            self.disable_torque()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._port.closePort()
        except Exception:  # noqa: BLE001
            pass

    def _raise_if_error(self, comm_result: int, dxl_error: int, op: str) -> None:
        if comm_result != 0:
            raise RuntimeError(f'{op} communication failed: {self._packet.getTxRxResult(comm_result)}')
        if dxl_error != 0:
            raise RuntimeError(f'{op} returned error: {self._packet.getRxPacketError(dxl_error)}')

    def set_operating_mode(self, mode: int) -> None:
        comm, err = self._packet.write1ByteTxRx(self._port, self._dxl_id, self.addr_operating_mode, int(mode))
        self._raise_if_error(comm, err, 'set_operating_mode')

    def enable_torque(self) -> None:
        comm, err = self._packet.write1ByteTxRx(self._port, self._dxl_id, self.addr_torque_enable, 1)
        self._raise_if_error(comm, err, 'enable_torque')

    def disable_torque(self) -> None:
        comm, err = self._packet.write1ByteTxRx(self._port, self._dxl_id, self.addr_torque_enable, 0)
        self._raise_if_error(comm, err, 'disable_torque')

    def write_goal_position(self, position_ticks: int) -> None:
        comm, err = self._packet.write4ByteTxRx(
            self._port, self._dxl_id, self.addr_goal_position, int(position_ticks) & 0xFFFFFFFF
        )
        self._raise_if_error(comm, err, 'write_goal_position')

    def write_goal_current(self, current_raw: int) -> None:
        # Goal Current is a signed 16-bit value in most Protocol 2.0 control tables.
        current_raw = int(max(-32768, min(32767, int(current_raw))))
        current_u16 = current_raw & 0xFFFF
        comm, err = self._packet.write2ByteTxRx(self._port, self._dxl_id, self.addr_goal_current, current_u16)
        self._raise_if_error(comm, err, 'write_goal_current')

    def read_present_position(self) -> int:
        data, comm, err = self._packet.read4ByteTxRx(self._port, self._dxl_id, self.addr_present_position)
        self._raise_if_error(comm, err, 'read_present_position')
        return int(data)

    def read_present_current(self) -> int:
        data, comm, err = self._packet.read2ByteTxRx(self._port, self._dxl_id, self.addr_present_current)
        self._raise_if_error(comm, err, 'read_present_current')
        raw = int(data) & 0xFFFF
        if raw >= 0x8000:
            raw -= 0x10000
        return int(raw)


class DynamixelGripperActionNode(Node):
    def __init__(self) -> None:
        super().__init__('gripper_dynamixel_action_node')

        # Optional motor configuration file loader.
        self.declare_parameter('motor_model', '')
        self.motor_model = str(self.get_parameter('motor_model').value)

        # Defaults used when goals are default-constructed (e.g. ros2 cli with {}).
        self.declare_parameter(self.motor_model + '.close_default', True)
        self.declare_parameter(self.motor_model + '.default_torque', 0.0)

        # If true, torque mode is used unless explicitly enabled in the goal.
        self.declare_parameter(self.motor_model + '.use_torque_mode', False)

        # Dynamixel Protocol 2.0 transport
        self.declare_parameter(self.motor_model + '.device_name', '/dev/ttyUSB0')
        self.declare_parameter(self.motor_model + '.baudrate', 57600)
        self.declare_parameter(self.motor_model + '.dxl_id', 1)

        # Control table addresses (defaults are common for many Protocol 2.0 models; override per model if needed).
        self.declare_parameter(self.motor_model + '.addr_operating_mode', 11)
        self.declare_parameter(self.motor_model + '.addr_torque_enable', 64)
        self.declare_parameter(self.motor_model + '.addr_goal_current', 102)
        self.declare_parameter(self.motor_model + '.addr_goal_position', 116)
        self.declare_parameter(self.motor_model + '.addr_present_current', 126)
        self.declare_parameter(self.motor_model + '.addr_present_position', 132)

        # Operating mode values
        self.declare_parameter(self.motor_model + '.operating_mode_current', 0)
        self.declare_parameter(self.motor_model + '.operating_mode_position', 3)
        self.declare_parameter(self.motor_model + '.operating_mode_current_based_position', 5)

        # Position scaling
        self.declare_parameter(self.motor_model + '.position_is_radians', False)
        self.declare_parameter(self.motor_model + '.open_position', 900.0)
        self.declare_parameter(self.motor_model + '.close_position', 2000.0)
        self.declare_parameter(self.motor_model + '.ticks_per_rev', 4096)
        self.declare_parameter(self.motor_model + '.gear_ratio', 1.0)
        self.declare_parameter(self.motor_model + '.direction', 1)
        self.declare_parameter(self.motor_model + '.zero_offset_ticks', 0)

        # Motion loop settings
        self.declare_parameter(self.motor_model + '.goal_tolerance_ticks', 20)
        self.declare_parameter(self.motor_model + '.motion_timeout_sec', 3.0)
        self.declare_parameter(self.motor_model + '.poll_rate_hz', 30.0)

        self._dxl: Optional[DynamixelProtocol2Driver] = None

        self._init_dynamixel()

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

        self.get_logger().info('Dynamixel gripper action node ready.')

    def destroy_node(self) -> bool:
        if self._dxl is not None:
            self._dxl.close()
        self._open_server.destroy()
        self._close_server.destroy()
        return super().destroy_node()

    def _init_dynamixel(self) -> None:
        try:
            self._dxl = DynamixelProtocol2Driver(
                device_name=str(self.get_parameter(self.motor_model + '.device_name').value),
                baudrate=int(self.get_parameter(self.motor_model + '.baudrate').value),
                dxl_id=int(self.get_parameter(self.motor_model + '.dxl_id').value),
                addr_operating_mode=int(self.get_parameter(self.motor_model + '.addr_operating_mode').value),
                addr_torque_enable=int(self.get_parameter(self.motor_model + '.addr_torque_enable').value),
                addr_goal_current=int(self.get_parameter(self.motor_model + '.addr_goal_current').value),
                addr_goal_position=int(self.get_parameter(self.motor_model + '.addr_goal_position').value),
                addr_present_current=int(self.get_parameter(self.motor_model + '.addr_present_current').value),
                addr_present_position=int(self.get_parameter(self.motor_model + '.addr_present_position').value),
                operating_mode_current=int(self.get_parameter(self.motor_model + '.operating_mode_current').value),
                operating_mode_position=int(self.get_parameter(self.motor_model + '.operating_mode_position').value),
                operating_mode_current_based_position=int(
                    self.get_parameter(self.motor_model + '.operating_mode_current_based_position').value
                ),
            )
            self.get_logger().info(
                f'Connected to Dynamixel id={int(self.get_parameter(self.motor_model + ".dxl_id").value)} '
                f'on {str(self.get_parameter(self.motor_model + ".device_name").value)} '
                f'@ {int(self.get_parameter(self.motor_model + ".baudrate").value)}'
            )
        except Exception as exc:  # noqa: BLE001
            self._dxl = None
            self.get_logger().error(f'Failed to initialize Dynamixel SDK driver: {exc}')

    def _position_to_ticks(self, position_value: float) -> int:
        if not bool(self.get_parameter(self.motor_model + '.position_is_radians').value):
            # Treat the parameter directly as ticks.
            return int(round(position_value))

        ticks_per_rev = float(self.get_parameter(self.motor_model + '.ticks_per_rev').value)
        gear_ratio = float(self.get_parameter(self.motor_model + '.gear_ratio').value)
        direction = int(self.get_parameter(self.motor_model + '.direction').value)
        zero = int(self.get_parameter(self.motor_model + '.zero_offset_ticks').value)

        ticks_per_rad = ticks_per_rev * gear_ratio / (2.0 * math.pi)
        return int(round(zero + direction * (position_value * ticks_per_rad)))

    def _goal_cb(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _resolve_torque(self, requested_torque: float) -> float:
        if abs(requested_torque) > 0.0:
            return float(requested_torque)
        return float(self.get_parameter(self.motor_model + '.default_torque').value)

    def _resolve_use_torque_mode(self, goal_flag: bool) -> bool:
        # Treat goal_flag as an explicit enable. If false, fall back to parameter default.
        return bool(goal_flag) or bool(self.get_parameter(self.motor_model + '.use_torque_mode').value)

    def _apply_position(self, target_position: float) -> int:
        if self._dxl is None:
            raise RuntimeError('Dynamixel driver is not initialized.')

        target_ticks = self._position_to_ticks(target_position)
        self._dxl.disable_torque()
        self._dxl.set_operating_mode(self._dxl.mode_position)
        self._dxl.enable_torque()
        self._dxl.write_goal_position(target_ticks)
        return target_ticks

    def _apply_torque(self, torque: float, target_position: Optional[float]) -> Optional[int]:
        if self._dxl is None:
            raise RuntimeError('Dynamixel driver is not initialized.')

        # `torque` is treated as a raw Goal Current value by default (model-specific units).
        goal_current = int(round(torque))

        self._dxl.disable_torque()

        if target_position is None:
            self._dxl.set_operating_mode(self._dxl.mode_current)
            self._dxl.enable_torque()
            self._dxl.write_goal_current(goal_current)
            return None

        target_ticks = self._position_to_ticks(float(target_position))
        self._dxl.set_operating_mode(self._dxl.mode_current_based_position)
        self._dxl.enable_torque()
        self._dxl.write_goal_current(goal_current)
        self._dxl.write_goal_position(target_ticks)
        return target_ticks

    def _run_motion_loop(self, goal_handle, feedback_cls, *, target_ticks: Optional[int]) -> bool:
        timeout = float(self.get_parameter(self.motor_model + '.motion_timeout_sec').value)
        poll_rate = float(self.get_parameter(self.motor_model + '.poll_rate_hz').value)
        poll_rate = 1.0 if poll_rate <= 0.0 else poll_rate
        sleep_sec = 1.0 / poll_rate
        tolerance = int(self.get_parameter(self.motor_model + '.goal_tolerance_ticks').value)

        start_time = time.monotonic()
        start_pos: Optional[int] = None
        if self._dxl is not None:
            try:
                start_pos = self._dxl.read_present_position()
            except Exception:  # noqa: BLE001
                start_pos = None

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False

            elapsed = time.monotonic() - start_time
            if elapsed > timeout:
                return False

            progress = 0.0
            if self._dxl is not None and target_ticks is not None:
                try:
                    pos = self._dxl.read_present_position()
                    err = abs(int(target_ticks) - int(pos))

                    if err <= tolerance:
                        goal_handle.publish_feedback(feedback_cls(progress=1.0))
                        return True

                    if start_pos is not None:
                        start_err = max(1, abs(int(target_ticks) - int(start_pos)))
                        progress = float(max(0.0, min(1.0, 1.0 - (err / float(start_err)))))
                except Exception:  # noqa: BLE001
                    progress = 0.0
            else:
                # Time-based progress fallback
                progress = float(max(0.0, min(1.0, elapsed / timeout)))

            goal_handle.publish_feedback(feedback_cls(progress=float(progress)))
            time.sleep(sleep_sec)

    def _execute_open(self, goal_handle) -> OpenGripper.Result:
        goal = goal_handle.request

        open_position = float(self.get_parameter(self.motor_model + '.open_position').value)
        torque = self._resolve_torque(float(goal.torque))
        use_torque_mode = self._resolve_use_torque_mode(bool(goal.use_torque_mode))

        try:
            if use_torque_mode:
                target_ticks = self._apply_torque(torque, open_position)
            else:
                target_ticks = self._apply_position(open_position)

            ok = self._run_motion_loop(goal_handle, OpenGripper.Feedback, target_ticks=target_ticks)
            if not ok:
                goal_handle.abort()
                return OpenGripper.Result(success=False, message='Open timed out or was canceled.')

            goal_handle.succeed()
            return OpenGripper.Result(success=True, message='Open command sent.')
        except Exception as exc:  # noqa: BLE001
            goal_handle.abort()
            return OpenGripper.Result(success=False, message=f'Open failed: {exc}')

    def _execute_close(self, goal_handle) -> CloseGripper.Result:
        goal = goal_handle.request

        close_default = bool(self.get_parameter(self.motor_model + '.close_default').value)
        close_requested = bool(goal.close) or close_default

        close_position = float(self.get_parameter(self.motor_model + '.close_position').value)
        torque = self._resolve_torque(float(goal.torque))
        use_torque_mode = self._resolve_use_torque_mode(bool(goal.use_torque_mode))

        if not close_requested:
            goal_handle.succeed()
            return CloseGripper.Result(success=True, message='Close goal flag was false; no action taken.')

        try:
            if use_torque_mode:
                target_ticks = self._apply_torque(torque, close_position)
            else:
                target_ticks = self._apply_position(close_position)

            ok = self._run_motion_loop(goal_handle, CloseGripper.Feedback, target_ticks=target_ticks)
            if not ok:
                goal_handle.abort()
                return CloseGripper.Result(success=False, message='Close timed out or was canceled.')

            goal_handle.succeed()
            return CloseGripper.Result(success=True, message='Close command sent.')
        except Exception as exc:  # noqa: BLE001
            goal_handle.abort()
            return CloseGripper.Result(success=False, message=f'Close failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamixelGripperActionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
