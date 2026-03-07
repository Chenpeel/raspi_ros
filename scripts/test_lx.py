#!/usr/bin/env python3
"""LX 总线舵机串口直测工具（不依赖 ROS 节点）。

用途:
1. 直接通过 pyserial 发/收 LX 协议帧，验证硬件链路
2. 打印 TX/RX 十六进制，定位“写成功但不动 / 读超时”问题

示例:
  python3 scripts/test_lx.py --port /dev/ttyAMA1 --id 21 read-id
  python3 scripts/test_lx.py --port /dev/ttyAMA1 --id 21 read-pos
  python3 scripts/test_lx.py --port /dev/ttyAMA1 --id 21 move --position 800 --time-ms 600
  python3 scripts/test_lx.py --port /dev/ttyAMA1 scan --start 1 --end 50
  python3 scripts/test_lx.py --port /dev/ttyAMA1 --id 21 demo
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "未安装 pyserial，请先执行: pip install pyserial"
    ) from exc


FRAME_HEADER = bytes([0x55, 0x55])

# LX 协议命令字（参考 docs/lx_bus_servo_protocol.pdf）
CMD_MOVE_TIME_WRITE = 0x01
CMD_ID_READ = 0x0E
CMD_OR_MOTOR_MODE_WRITE = 0x1D
CMD_POS_READ = 0x1C
CMD_LOAD_OR_UNLOAD_WRITE = 0x1F


def _hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def compute_checksum(data: Iterable[int]) -> int:
    return (~(sum(int(x) & 0xFF for x in data)) & 0xFF)


def split_u16(value: int) -> Tuple[int, int]:
    raw = int(value) & 0xFFFF
    return raw & 0xFF, (raw >> 8) & 0xFF


def split_i16(value: int) -> Tuple[int, int]:
    val = max(-32768, min(32767, int(value))) & 0xFFFF
    return val & 0xFF, (val >> 8) & 0xFF


def join_u16(low: int, high: int) -> int:
    return ((int(high) & 0xFF) << 8) | (int(low) & 0xFF)


def u16_to_i16(value: int) -> int:
    raw = int(value) & 0xFFFF
    return raw - 0x10000 if raw >= 0x8000 else raw


def build_packet(servo_id: int, cmd: int, params: Sequence[int] = ()) -> bytes:
    sid = max(0, min(253, int(servo_id)))
    payload = [int(p) & 0xFF for p in params]
    length = 3 + len(payload)  # length = LEN + CMD + PARAMS
    body = [sid, length, int(cmd) & 0xFF, *payload]
    checksum = compute_checksum(body)
    return bytes([*FRAME_HEADER, *body, checksum])


@dataclass
class LXFrame:
    raw: bytes
    servo_id: int
    cmd: int
    params: bytes


class LXSerialClient:
    """LX 串口客户端。"""

    def __init__(
        self,
        port: str,
        baud: int,
        timeout_s: float,
        debug: bool,
    ) -> None:
        self._debug = bool(debug)
        self._ser = serial.Serial(
            port=port,
            baudrate=int(baud),
            timeout=0.02,
            write_timeout=0.2,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self._default_timeout_s = float(timeout_s)
        self._buf = bytearray()

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    def __enter__(self) -> "LXSerialClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def send(self, packet: bytes) -> None:
        if self._debug:
            print(f"[TX] {_hex(packet)}")
        self._ser.write(packet)
        self._ser.flush()

    def clear_input(self) -> None:
        self._ser.reset_input_buffer()
        self._buf.clear()

    def _try_parse_frame(self) -> Optional[LXFrame]:
        while True:
            n = len(self._buf)
            if n < 6:
                return None

            idx = self._buf.find(FRAME_HEADER)
            if idx < 0:
                # 没有帧头，最多保留一个字节用于下轮拼接
                del self._buf[:-1]
                return None
            if idx > 0:
                del self._buf[:idx]
                n = len(self._buf)
                if n < 6:
                    return None

            # 现有 buf[0:2] == 0x55 0x55
            length = int(self._buf[3]) & 0xFF
            total_len = length + 3  # 含帧头和校验
            if total_len < 6:
                # 非法长度，丢弃第一个字节继续找
                del self._buf[0]
                continue
            if len(self._buf) < total_len:
                return None

            frame = bytes(self._buf[:total_len])
            expect = compute_checksum(frame[2:-1])
            if frame[-1] != expect:
                if self._debug:
                    print(f"[DROP] 校验失败: {_hex(frame)}")
                del self._buf[0]
                continue

            del self._buf[:total_len]
            return LXFrame(
                raw=frame,
                servo_id=frame[2],
                cmd=frame[4],
                params=frame[5:-1],
            )

    def read_frame(self, timeout_s: Optional[float] = None) -> Optional[LXFrame]:
        deadline = time.monotonic() + (self._default_timeout_s if timeout_s is None else timeout_s)
        while time.monotonic() < deadline:
            frame = self._try_parse_frame()
            if frame is not None:
                if self._debug:
                    print(f"[RX] {_hex(frame.raw)}")
                return frame

            chunk = self._ser.read(64)
            if chunk:
                self._buf.extend(chunk)
                continue

            time.sleep(0.001)

        return None

    def request(
        self,
        servo_id: int,
        cmd: int,
        params: Sequence[int] = (),
        expect_cmd: Optional[int] = None,
        tries: int = 1,
        timeout_s: Optional[float] = None,
    ) -> Optional[LXFrame]:
        expect_cmd = cmd if expect_cmd is None else int(expect_cmd)
        timeout_s = self._default_timeout_s if timeout_s is None else float(timeout_s)

        packet = build_packet(servo_id=servo_id, cmd=cmd, params=params)
        for attempt in range(1, max(1, int(tries)) + 1):
            self.clear_input()
            self.send(packet)
            frame = self.read_frame(timeout_s=timeout_s)
            if frame is None:
                if self._debug:
                    print(f"[TRY {attempt}] timeout")
                continue
            if frame.servo_id != int(servo_id):
                if self._debug:
                    print(
                        f"[TRY {attempt}] 忽略 frame: expect id={servo_id}, got id={frame.servo_id}"
                    )
                continue
            if frame.cmd != int(expect_cmd):
                if self._debug:
                    print(
                        f"[TRY {attempt}] 忽略 frame: expect cmd=0x{expect_cmd:02X}, got cmd=0x{frame.cmd:02X}"
                    )
                continue
            return frame

        return None

    def wait_reply(
        self,
        servo_id: int,
        expect_cmd: int,
        timeout_s: Optional[float] = None,
    ) -> Optional[LXFrame]:
        timeout_s = self._default_timeout_s if timeout_s is None else float(timeout_s)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.read_frame(timeout_s=max(0.001, deadline - time.monotonic()))
            if frame is None:
                return None
            if frame.servo_id != int(servo_id):
                if self._debug:
                    print(
                        f"[WAIT] 忽略 frame: expect id={servo_id}, got id={frame.servo_id}"
                    )
                continue
            if frame.cmd != int(expect_cmd):
                if self._debug:
                    print(
                        f"[WAIT] 忽略 frame: expect cmd=0x{expect_cmd:02X}, got cmd=0x{frame.cmd:02X}"
                    )
                continue
            return frame
        return None


def clamp(value: int, min_value: int, max_value: int) -> int:
    return max(min_value, min(max_value, int(value)))


def do_read_id(client: LXSerialClient, args: argparse.Namespace) -> int:
    frame = client.request(
        servo_id=args.id,
        cmd=CMD_ID_READ,
        tries=args.tries,
        timeout_s=args.timeout,
    )
    if frame is None:
        print("读取 ID 超时（无回包）")
        return 1
    if len(frame.params) < 1:
        print(f"ID 回包长度异常: {_hex(frame.raw)}")
        return 2
    servo_id = int(frame.params[0])
    print(f"读取成功: target={args.id}, response_id={servo_id}")
    return 0


def do_read_pos(client: LXSerialClient, args: argparse.Namespace) -> int:
    frame = client.request(
        servo_id=args.id,
        cmd=CMD_POS_READ,
        tries=args.tries,
        timeout_s=args.timeout,
    )
    if frame is None:
        print("读取位置超时（无回包）")
        return 1
    if len(frame.params) < 2:
        print(f"位置回包长度异常: {_hex(frame.raw)}")
        return 2
    pos_raw = join_u16(frame.params[0], frame.params[1])
    pos = u16_to_i16(pos_raw)
    print(f"读取成功: id={args.id}, position={pos} (raw={pos_raw})")
    return 0


def do_move(client: LXSerialClient, args: argparse.Namespace) -> int:
    position = clamp(args.position, 0, 1000)
    time_ms = clamp(args.time_ms, 0, 30000)
    pos_l, pos_h = split_u16(position)
    t_l, t_h = split_u16(time_ms)
    pkt = build_packet(args.id, CMD_MOVE_TIME_WRITE, [pos_l, pos_h, t_l, t_h])
    client.send(pkt)
    print(f"已发送 move_time_write: id={args.id}, position={position}, time_ms={time_ms}")

    if args.verify:
        time.sleep(max(0.0, args.verify_delay))
        frame = client.request(
            servo_id=args.id,
            cmd=CMD_POS_READ,
            tries=args.tries,
            timeout_s=args.timeout,
        )
        if frame is None or len(frame.params) < 2:
            print("verify 读取位置失败（无回包）")
            return 3
        pos_raw = join_u16(frame.params[0], frame.params[1])
        pos = u16_to_i16(pos_raw)
        print(f"verify: position={pos} (raw={pos_raw})")

    return 0


def do_load(client: LXSerialClient, args: argparse.Namespace) -> int:
    load = 1 if int(args.enable) else 0
    pkt = build_packet(args.id, CMD_LOAD_OR_UNLOAD_WRITE, [load])
    client.send(pkt)
    print(f"已发送 load_or_unload_write: id={args.id}, load={load}")
    return 0


def do_mode_servo(client: LXSerialClient, args: argparse.Namespace) -> int:
    # mode=0: servo mode, drive_mode=0, speed=0
    sp_l, sp_h = split_i16(0)
    pkt = build_packet(args.id, CMD_OR_MOTOR_MODE_WRITE, [0, 0, sp_l, sp_h])
    client.send(pkt)
    print(f"已发送 or_motor_mode_write: id={args.id}, mode=servo")
    return 0


def do_mode_motor(client: LXSerialClient, args: argparse.Namespace) -> int:
    drive_mode = 1 if int(args.drive_mode) else 0
    sp_l, sp_h = split_i16(args.speed)
    pkt = build_packet(args.id, CMD_OR_MOTOR_MODE_WRITE, [1, drive_mode, sp_l, sp_h])
    client.send(pkt)
    print(
        f"已发送 or_motor_mode_write: id={args.id}, mode=motor, "
        f"drive_mode={drive_mode}, speed={clamp(args.speed, -32768, 32767)}"
    )
    return 0


def do_scan(client: LXSerialClient, args: argparse.Namespace) -> int:
    start = clamp(args.start, 0, 253)
    end = clamp(args.end, 0, 253)
    if start > end:
        start, end = end, start

    found: List[int] = []
    print(f"开始扫描 ID 区间: [{start}, {end}]")
    for sid in range(start, end + 1):
        frame = client.request(
            servo_id=sid,
            cmd=CMD_ID_READ,
            tries=args.tries,
            timeout_s=args.timeout,
        )
        if frame is not None and len(frame.params) >= 1:
            found_id = int(frame.params[0])
            print(f"  - 响应: probe_id={sid}, response_id={found_id}")
            found.append(found_id)
        if args.delay > 0:
            time.sleep(args.delay)

    uniq = sorted(set(found))
    print(f"扫描结束, 检测到 {len(uniq)} 个 ID: {uniq}")
    return 0 if uniq else 1


def do_demo(client: LXSerialClient, args: argparse.Namespace) -> int:
    pos_a = clamp(args.pos_a, 0, 1000)
    pos_b = clamp(args.pos_b, 0, 1000)
    time_ms = clamp(args.time_ms, 0, 30000)

    print("步骤1: 上电(扭力使能)")
    do_load(client, argparse.Namespace(id=args.id, enable=1))
    time.sleep(0.08)

    print("步骤2: 切回舵机模式")
    do_mode_servo(client, argparse.Namespace(id=args.id))
    time.sleep(0.08)

    print(f"步骤3: move -> {pos_a}")
    do_move(
        client,
        argparse.Namespace(
            id=args.id,
            position=pos_a,
            time_ms=time_ms,
            verify=args.verify,
            verify_delay=args.verify_delay,
            tries=args.tries,
            timeout=args.timeout,
        ),
    )
    time.sleep(max(0.2, time_ms / 1000.0 + 0.2))

    print(f"步骤4: move -> {pos_b}")
    do_move(
        client,
        argparse.Namespace(
            id=args.id,
            position=pos_b,
            time_ms=time_ms,
            verify=args.verify,
            verify_delay=args.verify_delay,
            tries=args.tries,
            timeout=args.timeout,
        ),
    )
    return 0


def do_raw(client: LXSerialClient, args: argparse.Namespace) -> int:
    pkt = build_packet(args.id, args.cmd, args.params)
    client.clear_input()
    client.send(pkt)
    print(
        f"已发送 raw: id={args.id}, cmd=0x{args.cmd:02X}, "
        f"params={[int(p) & 0xFF for p in args.params]}"
    )

    if not args.expect:
        return 0

    frame = None
    for _ in range(max(1, args.tries)):
        frame = client.wait_reply(
            servo_id=args.id,
            expect_cmd=args.expect_cmd if args.expect_cmd is not None else args.cmd,
            timeout_s=args.timeout,
        )
        if frame is not None:
            break
    if frame is None:
        print("raw 读取超时（无回包）")
        return 1

    print(
        f"raw 回包: id={frame.servo_id}, cmd=0x{frame.cmd:02X}, "
        f"params={list(frame.params)}"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="LX 总线舵机串口测试工具（pyserial）"
    )
    parser.add_argument("--port", required=True, help="串口设备，例如 /dev/ttyAMA1")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument("--id", type=int, default=1, help="舵机 ID，默认 1")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.2,
        help="单次请求等待回包超时(秒)，默认 0.2",
    )
    parser.add_argument("--tries", type=int, default=2, help="读请求重试次数，默认 2")
    parser.add_argument("--debug", action="store_true", help="打印 TX/RX 十六进制日志")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("read-id", help="读取 ID")
    sub.add_parser("read-pos", help="读取当前位置")

    p_move = sub.add_parser("move", help="发送位置命令（move_time_write）")
    p_move.add_argument("--position", type=int, required=True, help="目标位置 [0,1000]")
    p_move.add_argument("--time-ms", type=int, default=600, help="运行时间(ms)，默认 600")
    p_move.add_argument("--verify", action="store_true", help="发送后读取一次位置")
    p_move.add_argument(
        "--verify-delay",
        type=float,
        default=0.12,
        help="发送后等待多久开始 verify 读位置(秒)，默认 0.12",
    )

    p_load = sub.add_parser("load", help="扭力开关（load_or_unload_write）")
    p_load.add_argument(
        "--enable",
        type=int,
        choices=[0, 1],
        required=True,
        help="1=上电(有扭力), 0=卸载(无扭力)",
    )

    sub.add_parser("mode-servo", help="切到舵机模式")

    p_mode_motor = sub.add_parser("mode-motor", help="切到电机模式")
    p_mode_motor.add_argument(
        "--drive-mode",
        type=int,
        choices=[0, 1],
        default=0,
        help="方向模式 0/1，默认 0",
    )
    p_mode_motor.add_argument(
        "--speed",
        type=int,
        default=0,
        help="电机模式速度（signed int16），默认 0",
    )

    p_scan = sub.add_parser("scan", help="扫描 ID（通过 id_read 探测回包）")
    p_scan.add_argument("--start", type=int, default=0, help="起始 ID，默认 0")
    p_scan.add_argument("--end", type=int, default=253, help="结束 ID，默认 253")
    p_scan.add_argument(
        "--delay",
        type=float,
        default=0.01,
        help="每个 ID 之间延迟(秒)，默认 0.01",
    )

    p_demo = sub.add_parser("demo", help="扭力使能 + 舵机模式 + 两点往返")
    p_demo.add_argument("--pos-a", type=int, default=120, help="点位 A，默认 120")
    p_demo.add_argument("--pos-b", type=int, default=880, help="点位 B，默认 880")
    p_demo.add_argument("--time-ms", type=int, default=600, help="单次运动时间(ms)")
    p_demo.add_argument("--verify", action="store_true", help="每次 move 后读取一次位置")
    p_demo.add_argument(
        "--verify-delay",
        type=float,
        default=0.12,
        help="verify 前等待时间(秒)，默认 0.12",
    )

    p_raw = sub.add_parser("raw", help="发送原始命令")
    p_raw.add_argument(
        "--cmd",
        type=lambda x: int(x, 0),
        required=True,
        help="命令字，支持十进制或 0x 前缀",
    )
    p_raw.add_argument(
        "--params",
        type=lambda x: int(x, 0),
        nargs="*",
        default=[],
        help="参数字节列表，支持十进制或 0x 前缀",
    )
    p_raw.add_argument("--expect", action="store_true", help="发送后等待一帧回包")
    p_raw.add_argument(
        "--expect-cmd",
        type=lambda x: int(x, 0),
        default=None,
        help="期望回包命令字（默认与 --cmd 相同）",
    )

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        with LXSerialClient(
            port=args.port,
            baud=args.baud,
            timeout_s=args.timeout,
            debug=args.debug,
        ) as client:
            if args.command == "read-id":
                return do_read_id(client, args)
            if args.command == "read-pos":
                return do_read_pos(client, args)
            if args.command == "move":
                return do_move(client, args)
            if args.command == "load":
                return do_load(client, args)
            if args.command == "mode-servo":
                return do_mode_servo(client, args)
            if args.command == "mode-motor":
                return do_mode_motor(client, args)
            if args.command == "scan":
                return do_scan(client, args)
            if args.command == "demo":
                return do_demo(client, args)
            if args.command == "raw":
                return do_raw(client, args)

            parser.error(f"未知 command: {args.command}")
            return 2
    except serial.SerialException as exc:
        print(f"串口错误: {exc}")
        return 10
    except KeyboardInterrupt:
        print("用户中断")
        return 130


if __name__ == "__main__":
    sys.exit(main())
