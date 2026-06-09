from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from dynamixel_sdk import PacketHandler, PortHandler
from rclpy.node import Node


@dataclass
class DynamixelServoConfig:
    motor_model: str = 'XM430'
    device_name: str = '/dev/ttyUSB0'
    baudrate: int = 57600
    dxl_id: int = 1
    addr_operating_mode: int = 11
    addr_torque_enable: int = 64
    addr_goal_current: int = 102
    addr_goal_position: int = 116
    addr_present_current: int = 126
    addr_present_position: int = 132
    operating_mode_current: int = 0
    operating_mode_position: int = 3
    operating_mode_current_based_position: int = 5
    position_is_radians: bool = False
    open_position: float = 900.0
    close_position: float = 2000.0
    ticks_per_rev: int = 4096
    gear_ratio: float = 1.0
    direction: int = 1
    zero_offset_ticks: int = 0
    goal_tolerance_ticks: int = 20
    motion_timeout_sec: float = 3.0
    poll_rate_hz: float = 30.0
    comm_retry_timeout_sec: float = 2.0
    comm_retry_initial_delay_sec: float = 0.05
    comm_retry_max_delay_sec: float = 0.5
    comm_retry_backoff: float = 1.7
    comm_retry_reinit_every: int = 5
    default_torque: float = 0.0
    use_torque_mode: bool = False
    close_default: bool = True

    @classmethod
    def from_node(cls, node: Node, motor_model: str) -> 'DynamixelServoConfig':
        prefix = motor_model + '.' if motor_model else ''

        def declare(name, default):
            return node.declare_parameter(prefix + name, default).value

        return cls(
            motor_model=motor_model,
            device_name=str(declare('device_name', '/dev/ttyUSB0')),
            baudrate=int(declare('baudrate', 57600)),
            dxl_id=int(declare('dxl_id', 1)),
            addr_operating_mode=int(declare('addr_operating_mode', 11)),
            addr_torque_enable=int(declare('addr_torque_enable', 64)),
            addr_goal_current=int(declare('addr_goal_current', 102)),
            addr_goal_position=int(declare('addr_goal_position', 116)),
            addr_present_current=int(declare('addr_present_current', 126)),
            addr_present_position=int(declare('addr_present_position', 132)),
            operating_mode_current=int(declare('operating_mode_current', 0)),
            operating_mode_position=int(declare('operating_mode_position', 3)),
            operating_mode_current_based_position=int(declare('operating_mode_current_based_position', 5)),
            position_is_radians=bool(declare('position_is_radians', False)),
            open_position=float(declare('open_position', 900.0)),
            close_position=float(declare('close_position', 2000.0)),
            ticks_per_rev=int(declare('ticks_per_rev', 4096)),
            gear_ratio=float(declare('gear_ratio', 1.0)),
            direction=int(declare('direction', 1)),
            zero_offset_ticks=int(declare('zero_offset_ticks', 0)),
            goal_tolerance_ticks=int(declare('goal_tolerance_ticks', 20)),
            motion_timeout_sec=float(declare('motion_timeout_sec', 3.0)),
            poll_rate_hz=float(declare('poll_rate_hz', 30.0)),
            comm_retry_timeout_sec=float(declare('comm_retry_timeout_sec', 2.0)),
            comm_retry_initial_delay_sec=float(declare('comm_retry_initial_delay_sec', 0.05)),
            comm_retry_max_delay_sec=float(declare('comm_retry_max_delay_sec', 0.5)),
            comm_retry_backoff=float(declare('comm_retry_backoff', 1.7)),
            comm_retry_reinit_every=int(declare('comm_retry_reinit_every', 5)),
            default_torque=float(declare('default_torque', 0.0)),
            use_torque_mode=bool(declare('use_torque_mode', False)),
            close_default=bool(declare('close_default', True)),
        )

    def position_to_ticks(self, value: float) -> int:
        if not self.position_is_radians:
            return int(round(value))

        ticks_per_rad = float(self.ticks_per_rev) * float(self.gear_ratio) / (2.0 * math.pi)
        return int(round(float(self.zero_offset_ticks) + float(self.direction) * (value * ticks_per_rad)))

    def ticks_to_command_units(self, ticks: int) -> float:
        if not self.position_is_radians:
            return float(ticks)

        ticks_per_rad = float(self.ticks_per_rev) * float(self.gear_ratio) / (2.0 * math.pi)
        if ticks_per_rad == 0.0 or self.direction == 0:
            return 0.0
        return float(ticks - self.zero_offset_ticks) / float(self.direction) / ticks_per_rad


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
                'dynamixel_sdk is not available. Install it using pip or apt.'
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
            self._port,
            self._dxl_id,
            self.addr_goal_position,
            int(position_ticks) & 0xFFFFFFFF,
        )
        self._raise_if_error(comm, err, 'write_goal_position')

    def write_goal_current(self, current_raw: int) -> None:
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


class DynamixelServo:
    def __init__(self, config: DynamixelServoConfig) -> None:
        self.config = config
        self._driver = DynamixelProtocol2Driver(
            device_name=config.device_name,
            baudrate=config.baudrate,
            dxl_id=config.dxl_id,
            addr_operating_mode=config.addr_operating_mode,
            addr_torque_enable=config.addr_torque_enable,
            addr_goal_current=config.addr_goal_current,
            addr_goal_position=config.addr_goal_position,
            addr_present_current=config.addr_present_current,
            addr_present_position=config.addr_present_position,
            operating_mode_current=config.operating_mode_current,
            operating_mode_position=config.operating_mode_position,
            operating_mode_current_based_position=config.operating_mode_current_based_position,
        )

    def close(self) -> None:
        self._driver.close()

    def enable_torque(self) -> None:
        self._driver.enable_torque()

    def disable_torque(self) -> None:
        self._driver.disable_torque()

    def set_operating_mode(self, mode: int) -> None:
        self._driver.set_operating_mode(mode)

    def write_goal_position(self, position_ticks: int) -> None:
        self._driver.write_goal_position(position_ticks)

    def write_goal_current(self, current_raw: int) -> None:
        self._driver.write_goal_current(current_raw)

    def read_present_position(self) -> int:
        return self._driver.read_present_position()

    def read_present_current(self) -> int:
        return self._driver.read_present_current()

    def position_to_ticks(self, value: float) -> int:
        return self.config.position_to_ticks(value)

    def ticks_to_command_units(self, ticks: int) -> float:
        return self.config.ticks_to_command_units(ticks)
