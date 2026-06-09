#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, Sequence

from dynamixel_sdk import PacketHandler, PortHandler


DEFAULT_ADDR_OPERATING_MODE = 11
DEFAULT_ADDR_TORQUE_ENABLE = 64
DEFAULT_ADDR_HARDWARE_ERROR = 70
DEFAULT_ADDR_PRESENT_CURRENT = 126
DEFAULT_ADDR_PRESENT_POSITION = 132


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description='Scan Dynamixel IDs and poll common Protocol 2.0 status registers.',
	)
	parser.add_argument('--device', default='/dev/ttyUSB0', help='Serial device path')
	parser.add_argument('--baudrate', type=int, default=57600, help='Serial baudrate')
	parser.add_argument(
		'--baudrate-sweep',
		nargs='+',
		type=int,
		help='Try multiple baudrates in sequence, for example: --baudrate-sweep 57600 115200 1000000',
	)
	parser.add_argument(
		'--common-baudrate-sweep',
		action='store_true',
		help='Try a built-in set of common Dynamixel baudrates',
	)
	parser.add_argument('--protocol', type=float, default=2.0, help='Dynamixel protocol version')
	parser.add_argument('--id', dest='ids', action='append', type=int, help='Target a specific servo ID; repeatable')
	parser.add_argument('--scan-start', type=int, default=0, help='Start of scan range when --id is not given')
	parser.add_argument('--scan-end', type=int, default=20, help='End of scan range when --id is not given')
	parser.add_argument('--interval', type=float, default=0.5, help='Polling interval in seconds')
	parser.add_argument(
		'--count',
		type=int,
		default=1,
		help='Number of polling iterations; use 0 for continuous polling',
	)
	parser.add_argument(
		'--no-poll',
		action='store_true',
		help='Only scan/ping IDs and print static info without polling registers',
	)
	parser.add_argument('--addr-operating-mode', type=int, default=DEFAULT_ADDR_OPERATING_MODE)
	parser.add_argument('--addr-torque-enable', type=int, default=DEFAULT_ADDR_TORQUE_ENABLE)
	parser.add_argument('--addr-hardware-error', type=int, default=DEFAULT_ADDR_HARDWARE_ERROR)
	parser.add_argument('--addr-present-current', type=int, default=DEFAULT_ADDR_PRESENT_CURRENT)
	parser.add_argument('--addr-present-position', type=int, default=DEFAULT_ADDR_PRESENT_POSITION)
	return parser.parse_args()


def target_baudrates(args: argparse.Namespace) -> list[int]:
	if args.baudrate_sweep:
		return list(dict.fromkeys(int(baudrate) for baudrate in args.baudrate_sweep))

	if args.common_baudrate_sweep:
		return [9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000]

	return [int(args.baudrate)]


def target_ids(args: argparse.Namespace) -> list[int]:
	if args.ids:
		return sorted(set(int(servo_id) for servo_id in args.ids))

	start = min(args.scan_start, args.scan_end)
	end = max(args.scan_start, args.scan_end)
	return list(range(start, end + 1))


def signed_u16(value: int) -> int:
	value &= 0xFFFF
	if value >= 0x8000:
		value -= 0x10000
	return value


def format_comm_error(packet: PacketHandler, comm_result: int, dxl_error: int) -> str:
	parts: list[str] = []
	if comm_result != 0:
		parts.append(packet.getTxRxResult(comm_result))
	if dxl_error != 0:
		parts.append(packet.getRxPacketError(dxl_error))
	return '; '.join(parts) if parts else 'unknown error'


def ping_id(port: PortHandler, packet: PacketHandler, servo_id: int) -> tuple[bool, str]:
	model_number, comm_result, dxl_error = packet.ping(port, servo_id)
	if comm_result == 0 and dxl_error == 0:
		return True, f'model={model_number}'
	return False, format_comm_error(packet, comm_result, dxl_error)


def read_u8(port: PortHandler, packet: PacketHandler, servo_id: int, address: int) -> tuple[int | None, str | None]:
	value, comm_result, dxl_error = packet.read1ByteTxRx(port, servo_id, address)
	if comm_result == 0 and dxl_error == 0:
		return int(value), None
	return None, format_comm_error(packet, comm_result, dxl_error)


def read_u16(port: PortHandler, packet: PacketHandler, servo_id: int, address: int) -> tuple[int | None, str | None]:
	value, comm_result, dxl_error = packet.read2ByteTxRx(port, servo_id, address)
	if comm_result == 0 and dxl_error == 0:
		return int(value), None
	return None, format_comm_error(packet, comm_result, dxl_error)


def read_u32(port: PortHandler, packet: PacketHandler, servo_id: int, address: int) -> tuple[int | None, str | None]:
	value, comm_result, dxl_error = packet.read4ByteTxRx(port, servo_id, address)
	if comm_result == 0 and dxl_error == 0:
		return int(value), None
	return None, format_comm_error(packet, comm_result, dxl_error)


def discover_servos(port: PortHandler, packet: PacketHandler, servo_ids: Sequence[int]) -> list[int]:
	print(f'Scanning IDs on {port.getPortName()} @ requested baudrate...')
	found: list[int] = []
	for servo_id in servo_ids:
		ok, detail = ping_id(port, packet, servo_id)
		if ok:
			found.append(servo_id)
			print(f'  ID {servo_id}: online ({detail})')
		else:
			print(f'  ID {servo_id}: no response ({detail})')
	return found


def scan_at_baudrate(
	port: PortHandler,
	packet: PacketHandler,
	servo_ids: Sequence[int],
	baudrate: int,
) -> list[int]:
	if not port.setBaudRate(baudrate):
		print(f'Failed to set baudrate {baudrate} on {port.getPortName()}', file=sys.stderr)
		return []

	print(f'\n=== Baudrate {baudrate} ===')
	return discover_servos(port, packet, servo_ids)


def print_poll_snapshot(
	port: PortHandler,
	packet: PacketHandler,
	servo_ids: Iterable[int],
	args: argparse.Namespace,
) -> None:
	timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
	print(f'[{timestamp}] Poll snapshot')
	for servo_id in servo_ids:
		mode, mode_err = read_u8(port, packet, servo_id, args.addr_operating_mode)
		torque, torque_err = read_u8(port, packet, servo_id, args.addr_torque_enable)
		hw_err, hw_err_msg = read_u8(port, packet, servo_id, args.addr_hardware_error)
		current_raw, current_err = read_u16(port, packet, servo_id, args.addr_present_current)
		position, position_err = read_u32(port, packet, servo_id, args.addr_present_position)

		current_text = str(signed_u16(current_raw)) if current_raw is not None else f'ERR({current_err})'
		mode_text = str(mode) if mode is not None else f'ERR({mode_err})'
		torque_text = str(torque) if torque is not None else f'ERR({torque_err})'
		hw_err_text = str(hw_err) if hw_err is not None else f'ERR({hw_err_msg})'
		position_text = str(position) if position is not None else f'ERR({position_err})'

		print(
			f'  ID {servo_id}: '
			f'mode={mode_text} torque_enable={torque_text} '
			f'hw_error={hw_err_text} present_current={current_text} '
			f'present_position={position_text}'
		)


def main() -> int:
	args = parse_args()
	servo_ids = target_ids(args)
	baudrates = target_baudrates(args)

	port = PortHandler(args.device)
	packet = PacketHandler(args.protocol)

	if not port.openPort():
		print(f'Failed to open port: {args.device}', file=sys.stderr)
		return 1

	try:
		results: dict[int, list[int]] = {}
		for baudrate in baudrates:
			results[baudrate] = scan_at_baudrate(port, packet, servo_ids, baudrate)

		found_baudrates = {baudrate: found_ids for baudrate, found_ids in results.items() if found_ids}
		if not found_baudrates:
			print('No responding Dynamixel servos found.')
			return 2

		print('\nSummary:')
		for baudrate, found_ids in found_baudrates.items():
			ids_text = ', '.join(str(servo_id) for servo_id in found_ids)
			print(f'  baudrate {baudrate}: IDs {ids_text}')

		if args.no_poll:
			return 0

		poll_baudrate = next(iter(found_baudrates))
		found_ids = found_baudrates[poll_baudrate]
		if not port.setBaudRate(poll_baudrate):
			print(f'Failed to set baudrate {poll_baudrate} on {args.device}', file=sys.stderr)
			return 1

		print(f'\nPolling IDs at baudrate {poll_baudrate}')

		iteration = 0
		while args.count == 0 or iteration < args.count:
			print_poll_snapshot(port, packet, found_ids, args)
			iteration += 1
			if args.count == 0 or iteration < args.count:
				time.sleep(max(0.0, args.interval))
		return 0
	finally:
		try:
			port.closePort()
		except Exception:
			pass


if __name__ == '__main__':
	raise SystemExit(main())
