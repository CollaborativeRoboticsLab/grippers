from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Optional

from dynamixel_sdk import PacketHandler, PortHandler
from rclpy.node import Node


@dataclass
class DynamixelServoConfig:
    motor_model: str = 'XM430'
    device_name: str = '/dev/ttyUSB0'
    baudrate: int = 57600
    servo_id: int = 1
    addr_operating_mode: int = 11
    addr_current_limit: int = 38
    addr_torque_enable: int = 64
    addr_hardware_error_status: int = 70
    addr_goal_current: int = 102
    addr_goal_position: int = 116
    addr_present_current: int = 126
    addr_present_position: int = 132
    addr_present_input_voltage: int = 144
    addr_present_temperature: int = 146
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
    torque_reached_min_duration_sec: float = 0.5
    poll_rate_hz: float = 30.0
    comm_retry_timeout_sec: float = 2.0
    comm_retry_initial_delay_sec: float = 0.05
    comm_retry_max_delay_sec: float = 0.5
    comm_retry_backoff: float = 1.7
    comm_retry_reinit_every: int = 5
    torque_per_current_unit: float = 1.0
    min_current_unit: int = 0
    max_current_unit: int = 1193
    control_torque: float = 0.0
    safety_torque_limit: float = 0.0
    stall_torque: float = 0.0
    stall_current: float = 0.0
    use_torque_mode: bool = False
    close_default: bool = True

    def __post_init__(self) -> None:
        self.min_current_unit = int(self.min_current_unit)
        self.max_current_unit = int(self.max_current_unit)
        if self.min_current_unit < 0:
            raise ValueError('min_current_unit must be greater than or equal to 0.')
        if self.max_current_unit < self.min_current_unit:
            raise ValueError('max_current_unit must be greater than or equal to min_current_unit.')
        if self.stall_torque > 0.0 and self.safety_torque_limit > self.stall_torque:
            raise ValueError('safety_torque_limit must be less than or equal to stall_torque.')

    @classmethod
    def from_node(cls, node: Node, motor_model: str) -> 'DynamixelServoConfig':
        prefix = motor_model + '.' if motor_model else ''

        def declare(name, default):
            return node.declare_parameter(prefix + name, default).value

        return cls(
            motor_model=motor_model,
            device_name=str(declare('device_name', '/dev/ttyUSB0')),
            baudrate=int(declare('baudrate', 57600)),
            servo_id=int(declare('servo_id', 1)),
            addr_operating_mode=int(declare('addr_operating_mode', 11)),
            addr_current_limit=int(declare('addr_current_limit', 38)),
            addr_torque_enable=int(declare('addr_torque_enable', 64)),
            addr_hardware_error_status=int(declare('addr_hardware_error_status', 70)),
            addr_goal_current=int(declare('addr_goal_current', 102)),
            addr_goal_position=int(declare('addr_goal_position', 116)),
            addr_present_current=int(declare('addr_present_current', 126)),
            addr_present_position=int(declare('addr_present_position', 132)),
            addr_present_input_voltage=int(declare('addr_present_input_voltage', 144)),
            addr_present_temperature=int(declare('addr_present_temperature', 146)),
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
            torque_reached_min_duration_sec=float(declare('torque_reached_min_duration_sec', 0.5)),
            poll_rate_hz=float(declare('poll_rate_hz', 30.0)),
            comm_retry_timeout_sec=float(declare('comm_retry_timeout_sec', 2.0)),
            comm_retry_initial_delay_sec=float(declare('comm_retry_initial_delay_sec', 0.05)),
            comm_retry_max_delay_sec=float(declare('comm_retry_max_delay_sec', 0.5)),
            comm_retry_backoff=float(declare('comm_retry_backoff', 1.7)),
            comm_retry_reinit_every=int(declare('comm_retry_reinit_every', 5)),
            torque_per_current_unit=float(declare('torque_per_current_unit', 1.0)),
            min_current_unit=int(declare('min_current_unit', 0)),
            max_current_unit=int(declare('max_current_unit', 1193)),
            control_torque=float(declare('control_torque', 0.0)),
            safety_torque_limit=float(declare('safety_torque_limit', 0.0)),
            stall_torque=float(declare('stall_torque', 0.0)),
            stall_current=float(declare('stall_current', 0.0)),
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

    def torque_to_current_raw(self, torque: float) -> int:
        scale = float(self.torque_per_current_unit)
        if math.isclose(scale, 0.0):
            raw = int(round(float(torque)))
        else:
            raw = int(round(float(torque) / scale))
        return self.clamp_current_raw(raw)

    def current_raw_to_torque(self, current_raw: int) -> float:
        return float(current_raw) * float(self.torque_per_current_unit)

    def clamp_current_raw(self, current_raw: int) -> int:
        raw = int(current_raw)
        if raw == 0:
            return 0

        magnitude = min(self.max_current_unit, abs(raw))
        if self.min_current_unit > 0:
            magnitude = max(self.min_current_unit, magnitude)
        return -magnitude if raw < 0 else magnitude


class DynamixelProtocol2Driver:
    def __init__(
        self,
        *,
        device_name: str,
        baudrate: int,
        servo_id: int,
        addr_operating_mode: int,
        addr_current_limit: int,
        addr_torque_enable: int,
        addr_hardware_error_status: int,
        addr_goal_current: int,
        addr_goal_position: int,
        addr_present_current: int,
        addr_present_position: int,
        addr_present_input_voltage: int,
        addr_present_temperature: int,
        operating_mode_current: int,
        operating_mode_position: int,
        operating_mode_current_based_position: int,
        comm_retry_timeout_sec: float,
        comm_retry_initial_delay_sec: float,
        comm_retry_max_delay_sec: float,
        comm_retry_backoff: float,
        comm_retry_reinit_every: int,
    ) -> None:
        if PortHandler is None or PacketHandler is None:
            raise RuntimeError(
                'dynamixel_sdk is not available. Install it using pip or apt.'
            )

        self._servo_id = int(servo_id)
        self._port = PortHandler(device_name)
        self._packet = PacketHandler(2.0)
        self._device_name = device_name
        self._baudrate = int(baudrate)
        self._comm_retry_timeout_sec = max(0.0, float(comm_retry_timeout_sec))
        self._comm_retry_initial_delay_sec = max(0.0, float(comm_retry_initial_delay_sec))
        self._comm_retry_max_delay_sec = max(0.0, float(comm_retry_max_delay_sec))
        self._comm_retry_backoff = max(1.0, float(comm_retry_backoff))
        self._comm_retry_reinit_every = max(0, int(comm_retry_reinit_every))

        self.addr_operating_mode = int(addr_operating_mode)
        self.addr_current_limit = int(addr_current_limit)
        self.addr_torque_enable = int(addr_torque_enable)
        self.addr_hardware_error_status = int(addr_hardware_error_status)
        self.addr_goal_current = int(addr_goal_current)
        self.addr_goal_position = int(addr_goal_position)
        self.addr_present_current = int(addr_present_current)
        self.addr_present_position = int(addr_present_position)
        self.addr_present_input_voltage = int(addr_present_input_voltage)
        self.addr_present_temperature = int(addr_present_temperature)

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

    def _reopen(self) -> None:
        try:
            self._port.closePort()
        except Exception:  # noqa: BLE001
            pass
        self._open()

    def _is_retryable_packet_error(self, dxl_error: int) -> bool:
        if dxl_error == 0:
            return False
        error_text = self._packet.getRxPacketError(dxl_error)
        return 'CRC' in error_text or 'corrupt' in error_text.lower()

    def _normalize_sdk_result(self, op: str, result: tuple) -> tuple[Optional[int], int, int]:
        if len(result) == 2:
            comm_result, dxl_error = result
            return None, int(comm_result), int(dxl_error)
        if len(result) == 3:
            data, comm_result, dxl_error = result
            return int(data), int(comm_result), int(dxl_error)
        raise RuntimeError(f'{op} returned an unexpected SDK result shape: {len(result)} values')

    def _with_retry(self, op: str, call: Callable[[], tuple]) -> tuple[Optional[int], int, int]:
        deadline = time.monotonic() + self._comm_retry_timeout_sec
        delay = self._comm_retry_initial_delay_sec
        attempt = 0
        last_error: Optional[Exception] = None

        while True:
            try:
                result = self._normalize_sdk_result(op, call())
                _, comm_result, dxl_error = result
                if comm_result == 0 and not self._is_retryable_packet_error(dxl_error):
                    return result

                if comm_result == 0:
                    self._raise_if_error(comm_result, dxl_error, op)
            except IndexError as exc:
                last_error = RuntimeError(f'{op} communication failed: {exc}')
            except RuntimeError as exc:
                last_error = exc
            except Exception:  # noqa: BLE001
                raise

            attempt += 1
            if time.monotonic() >= deadline:
                if last_error is not None:
                    raise last_error
                raise RuntimeError(f'{op} communication failed after retries.')

            if self._comm_retry_reinit_every > 0 and attempt % self._comm_retry_reinit_every == 0:
                self._reopen()

            if delay > 0.0:
                time.sleep(delay)
            if self._comm_retry_max_delay_sec > 0.0:
                delay = min(
                    self._comm_retry_max_delay_sec,
                    max(self._comm_retry_initial_delay_sec, delay * self._comm_retry_backoff),
                )

    def _raise_if_error(self, comm_result: int, dxl_error: int, op: str) -> None:
        if comm_result != 0:
            raise RuntimeError(f'{op} communication failed: {self._packet.getTxRxResult(comm_result)}')
        if dxl_error != 0:
            details = self._build_error_details()
            suffix = f' ({details})' if details else ''
            raise RuntimeError(f'{op} returned error: {self._packet.getRxPacketError(dxl_error)}{suffix}')

    def _read_register_1byte_no_raise(self, address: int) -> tuple[Optional[int], Optional[str]]:
        try:
            data, comm_result, dxl_error = self._normalize_sdk_result(
                'read_register_1byte_no_raise',
                self._packet.read1ByteTxRx(self._port, self._servo_id, int(address)),
            )
        except Exception as exc:  # noqa: BLE001
            return None, str(exc)

        if comm_result != 0:
            return None, self._packet.getTxRxResult(comm_result)
        if dxl_error != 0:
            return None, self._packet.getRxPacketError(dxl_error)
        return int(data), None

    def _decode_hardware_error_status(self, status: int) -> str:
        known_bits = {
            0: 'input_voltage',
            2: 'overheating',
            3: 'motor_encoder',
            4: 'electrical_shock',
            5: 'overload',
        }
        names: list[str] = []
        remaining = int(status)
        for bit, name in known_bits.items():
            if status & (1 << bit):
                names.append(name)
                remaining &= ~(1 << bit)
        bit_index = 0
        while remaining:
            if remaining & 1:
                names.append(f'bit{bit_index}')
            remaining >>= 1
            bit_index += 1
        return '|'.join(names) if names else 'none'

    def _build_error_details(self) -> str:
        parts: list[str] = []

        hardware_status, hardware_error = self._read_register_1byte_no_raise(self.addr_hardware_error_status)
        if hardware_status is not None:
            parts.append(
                'hardware_error_status=' +
                f'0x{hardware_status:02X}:{self._decode_hardware_error_status(hardware_status)}'
            )
        elif hardware_error is not None:
            parts.append(f'hardware_error_status_unavailable={hardware_error}')

        present_voltage_raw, present_voltage_error = self._read_register_1byte_no_raise(self.addr_present_input_voltage)
        if present_voltage_raw is not None:
            parts.append(f'present_input_voltage={present_voltage_raw / 10.0:.1f}V')
        elif present_voltage_error is not None:
            parts.append(f'present_input_voltage_unavailable={present_voltage_error}')

        present_temperature, present_temperature_error = self._read_register_1byte_no_raise(self.addr_present_temperature)
        if present_temperature is not None:
            parts.append(f'present_temperature={present_temperature}C')
        elif present_temperature_error is not None:
            parts.append(f'present_temperature_unavailable={present_temperature_error}')

        return ', '.join(parts)

    def set_operating_mode(self, mode: int) -> None:
        _, comm, err = self._with_retry(
            'set_operating_mode',
            lambda: self._packet.write1ByteTxRx(self._port, self._servo_id, self.addr_operating_mode, int(mode)),
        )
        self._raise_if_error(comm, err, 'set_operating_mode')

    def read_current_limit(self) -> int:
        data, comm, err = self._with_retry(
            'read_current_limit',
            lambda: self._packet.read2ByteTxRx(self._port, self._servo_id, self.addr_current_limit),
        )
        self._raise_if_error(comm, err, 'read_current_limit')
        return int(data)

    def enable_torque(self) -> None:
        _, comm, err = self._with_retry(
            'enable_torque',
            lambda: self._packet.write1ByteTxRx(self._port, self._servo_id, self.addr_torque_enable, 1),
        )
        self._raise_if_error(comm, err, 'enable_torque')

    def disable_torque(self) -> None:
        _, comm, err = self._with_retry(
            'disable_torque',
            lambda: self._packet.write1ByteTxRx(self._port, self._servo_id, self.addr_torque_enable, 0),
        )
        self._raise_if_error(comm, err, 'disable_torque')

    def write_goal_position(self, position_ticks: int) -> None:
        _, comm, err = self._with_retry(
            'write_goal_position',
            lambda: self._packet.write4ByteTxRx(
                self._port,
                self._servo_id,
                self.addr_goal_position,
                int(position_ticks) & 0xFFFFFFFF,
            ),
        )
        self._raise_if_error(comm, err, 'write_goal_position')

    def write_goal_current(self, current_raw: int) -> None:
        current_raw = int(current_raw)
        current_u16 = current_raw & 0xFFFF
        _, comm, err = self._with_retry(
            'write_goal_current',
            lambda: self._packet.write2ByteTxRx(self._port, self._servo_id, self.addr_goal_current, current_u16),
        )
        self._raise_if_error(comm, err, 'write_goal_current')

    def read_present_position(self) -> int:
        data, comm, err = self._with_retry(
            'read_present_position',
            lambda: self._packet.read4ByteTxRx(self._port, self._servo_id, self.addr_present_position),
        )
        self._raise_if_error(comm, err, 'read_present_position')
        return int(data)

    def read_present_current(self) -> int:
        data, comm, err = self._with_retry(
            'read_present_current',
            lambda: self._packet.read2ByteTxRx(self._port, self._servo_id, self.addr_present_current),
        )
        self._raise_if_error(comm, err, 'read_present_current')
        raw = int(data) & 0xFFFF
        if raw >= 0x8000:
            raw -= 0x10000
        return int(raw)


class DynamixelServo:
    def __init__(self, config: DynamixelServoConfig, warn: Optional[Callable[[str], None]] = None) -> None:
        self.config = config
        self._warn = warn
        self._effective_max_current_unit = int(config.max_current_unit)
        self._driver = DynamixelProtocol2Driver(
            device_name=config.device_name,
            baudrate=config.baudrate,
            servo_id=config.servo_id,
            addr_operating_mode=config.addr_operating_mode,
            addr_current_limit=config.addr_current_limit,
            addr_torque_enable=config.addr_torque_enable,
            addr_hardware_error_status=config.addr_hardware_error_status,
            addr_goal_current=config.addr_goal_current,
            addr_goal_position=config.addr_goal_position,
            addr_present_current=config.addr_present_current,
            addr_present_position=config.addr_present_position,
            addr_present_input_voltage=config.addr_present_input_voltage,
            addr_present_temperature=config.addr_present_temperature,
            operating_mode_current=config.operating_mode_current,
            operating_mode_position=config.operating_mode_position,
            operating_mode_current_based_position=config.operating_mode_current_based_position,
            comm_retry_timeout_sec=config.comm_retry_timeout_sec,
            comm_retry_initial_delay_sec=config.comm_retry_initial_delay_sec,
            comm_retry_max_delay_sec=config.comm_retry_max_delay_sec,
            comm_retry_backoff=config.comm_retry_backoff,
            comm_retry_reinit_every=config.comm_retry_reinit_every,
        )
        self._effective_max_current_unit = self._read_effective_max_current_unit()

    def close(self) -> None:
        self._driver.close()

    def enable_torque(self) -> None:
        self._driver.enable_torque()

    def disable_torque(self) -> None:
        self._driver.disable_torque()

    def _read_effective_max_current_unit(self) -> int:
        hardware_limit = int(self._driver.read_current_limit())
        software_limit = int(self.config.max_current_unit)
        if hardware_limit <= 0:
            if self._warn is not None:
                self._warn(
                    f'Dynamixel Current Limit register ({self.config.addr_current_limit}) is {hardware_limit}; '
                    'torque/current commands will be clamped to 0.'
                )
            return 0
        if software_limit != hardware_limit and self._warn is not None:
            self._warn(
                f'Dynamixel Current Limit register ({self.config.addr_current_limit})={hardware_limit} differs from '
                f'configured max_current_unit={software_limit}; using hardware register value.'
            )
        return hardware_limit

    def _clamp_current_raw(self, current_raw: int) -> int:
        raw = int(current_raw)
        if raw == 0:
            return 0

        magnitude = min(self._effective_max_current_unit, abs(raw))
        if self.config.min_current_unit > 0 and self._effective_max_current_unit >= self.config.min_current_unit:
            magnitude = max(int(self.config.min_current_unit), magnitude)
        return -magnitude if raw < 0 else magnitude

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

    def read_present_torque(self) -> float:
        return self.config.current_raw_to_torque(self.read_present_current())

    def position_to_ticks(self, value: float) -> int:
        return self.config.position_to_ticks(value)

    def ticks_to_command_units(self, ticks: int) -> float:
        return self.config.ticks_to_command_units(ticks)

    def torque_to_current_raw(self, torque: float) -> int:
        scale = float(self.config.torque_per_current_unit)
        if math.isclose(scale, 0.0):
            raw = int(round(float(torque)))
        else:
            raw = int(round(float(torque) / scale))
        return self._clamp_current_raw(raw)

    def current_raw_to_torque(self, current_raw: int) -> float:
        return self.config.current_raw_to_torque(current_raw)
