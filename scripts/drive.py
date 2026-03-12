#!/usr/bin/env -S uv run
# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "pyserial>=3.5",
#   "websockets>=12.0",
# ]
# ///
"""独立 WebSocket -> LX 舵机驱动。

用途:
1. 直接起一个 WebSocket 服务端，接收与项目现有协议兼容的消息
2. 解析总线舵机指令后，直接通过 ttyAMA1 向 LX 舵机发送串口帧
3. 绕过 ROS，仅保留 WebSocket 解析与 LX 串口驱动两层

默认行为:
- 监听: ws://0.0.0.0:9105
- 设备 ID: robot
- LX 串口: /dev/ttyAMA1
- 允许舵机 ID: 21-37

示例:
  uv run scripts/drive.py
  uv run scripts/drive.py --debug
  uv run scripts/drive.py --ids 21-37 --serial /dev/ttyAMA1
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import signal
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

ROOT = Path(__file__).resolve().parents[1]
for package_root in (ROOT / "src" / "hardware" / "servo_hardware",):
    package_root_str = str(package_root)
    if package_root_str not in sys.path:
        sys.path.insert(0, package_root_str)

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "缺少 pyserial，建议使用: uv run scripts/drive.py ..."
    ) from exc

try:
    from websockets.exceptions import ConnectionClosed
    from websockets.asyncio.server import serve
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "缺少 websockets，建议使用: uv run scripts/drive.py ..."
    ) from exc

from protocols.lx_protocol import LXBusServoProtocol


LOGGER = logging.getLogger("drive")
ROBOT_DEVICE_ID = "ros2-robot-device"
DEFAULT_IDS = "21-37"


def clamp_int(value: int, min_value: int, max_value: int) -> int:
    """整数限幅。"""
    return max(int(min_value), min(int(max_value), int(value)))


def clamp_float(value: float, min_value: float, max_value: float) -> float:
    """浮点数限幅。"""
    return max(float(min_value), min(float(max_value), float(value)))


def parse_servo_ids(spec: str) -> list[int]:
    """解析 21-37,40,42-43 这样的舵机 ID 配置。"""
    servo_ids: set[int] = set()
    for chunk in str(spec or "").split(","):
        item = chunk.strip()
        if not item:
            continue
        if "-" in item:
            start_raw, end_raw = item.split("-", 1)
            start = int(start_raw)
            end = int(end_raw)
            if start > end:
                start, end = end, start
            for servo_id in range(start, end + 1):
                servo_ids.add(clamp_int(servo_id, 0, 253))
            continue
        servo_ids.add(clamp_int(int(item), 0, 253))

    if not servo_ids:
        raise ValueError("至少需要一个舵机 ID")
    return sorted(servo_ids)


def pulse_to_centered_angle(pulse: float) -> float:
    """统一脉宽 500~2500us -> 中心角 [-90, 90]。"""
    pulse = clamp_float(pulse, 500.0, 2500.0)
    return (pulse - 500.0) * (180.0 / 2000.0) - 90.0


def centered_angle_to_lx_position(angle_deg: float) -> int:
    """中心角 [-90, 90] -> LX 原生位置 [125, 875]。

    约定与本项目当前 LX 方向保持一致:
    +90deg -> 125
      0deg -> 500
    -90deg -> 875
    """
    angle_deg = clamp_float(angle_deg, -90.0, 90.0)
    return int(round(500.0 - angle_deg * 750.0 / 180.0))


def lx_position_to_centered_angle(position: int) -> float:
    """LX 原生位置 [125, 875] -> 中心角 [-90, 90]。"""
    position = clamp_int(position, 125, 875)
    return (500.0 - float(position)) * 180.0 / 750.0


def setup_logging(debug: bool) -> None:
    """初始化日志。"""
    level = logging.DEBUG if debug else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )


class WSMessageParser:
    """独立脚本专用的轻量 WS 消息解析器。"""

    def __init__(self, debug: bool = False) -> None:
        self.debug = bool(debug)

    def parse_message(self, raw_message: str) -> Optional[Any]:
        """解析原始 JSON 字符串。"""
        try:
            data = json.loads(raw_message)
            if self.debug:
                LOGGER.debug("WS parse success: %s", data)
            return data
        except json.JSONDecodeError:
            LOGGER.exception("WS JSON 解析失败")
            return None

    @staticmethod
    def get_message_type(data: Any) -> str:
        """推断消息类型。"""
        if not isinstance(data, dict):
            return "unknown"

        msg_type = (
            data.get("type")
            or data.get("Type")
            or data.get("TYPE")
            or ""
        )
        msg_type = str(msg_type).strip().lower()
        if msg_type in ("action", "bvh_play"):
            return "bvh_play"
        if msg_type:
            return msg_type

        if WSMessageParser._is_servo_control(data):
            return "servo_control"
        if "timestamp" in data and "status" in data:
            return "heartbeat"
        return "unknown"

    @staticmethod
    def _is_servo_control(data: dict[str, Any]) -> bool:
        """判断一条 dict 是否像舵机命令。"""
        if not isinstance(data, dict):
            return False
        if "web_servo" in data and isinstance(data.get("web_servo"), dict):
            return True
        if all(key in data for key in ("b", "c", "p")):
            return True
        return any(
            key in data
            for key in ("servo_id", "id", "channel", "position", "angle", "pulse")
        )

    def parse_servo_control(self, data: dict[str, Any]) -> Optional[dict[str, Any]]:
        """解析 bus/pca 舵机命令。"""
        if not isinstance(data, dict):
            return None

        if "web_servo" in data:
            return self._parse_web_servo_payload(data.get("web_servo"), data)

        if any(key in data for key in ("is_bus_servo", "servo_id", "position", "angle")):
            parsed = self._parse_web_servo_payload(data, None)
            if parsed is not None:
                return parsed

        if all(key in data for key in ("b", "c", "p")):
            return self._parse_bcp_protocol(data)

        return self._parse_full_format(data)

    def _parse_bcp_protocol(self, data: dict[str, Any]) -> Optional[dict[str, Any]]:
        """解析 {b,c,p,s} 简写协议。"""
        try:
            b_flag = int(data.get("b", 0))
            servo_id = int(data.get("c", 0))
            raw_position = int(data.get("p", 0))
            speed = int(data.get("s", 100))
            if b_flag == -1:
                return {
                    "servo_type": "bus",
                    "servo_id": servo_id,
                    "position": raw_position,
                    "speed": speed,
                    "port": int(data.get("port", 0)),
                }
            if b_flag == 0:
                return {
                    "servo_type": "pca",
                    "servo_id": servo_id,
                    "position": raw_position,
                    "port": int(data.get("port", 0)),
                }
            return None
        except (TypeError, ValueError):
            LOGGER.exception("解析 BCP 协议失败: %s", data)
            return None

    def _parse_web_servo_payload(
        self,
        web_servo: Any,
        outer: Optional[dict[str, Any]],
    ) -> Optional[dict[str, Any]]:
        """解析标准 web_servo payload。"""
        if not isinstance(web_servo, dict):
            return None

        try:
            is_bus_servo = None
            raw_flag = web_servo.get("is_bus_servo")
            if isinstance(raw_flag, bool):
                is_bus_servo = raw_flag
            elif isinstance(raw_flag, (int, float)):
                is_bus_servo = bool(int(raw_flag))
            elif isinstance(raw_flag, str):
                flag = raw_flag.strip().lower()
                if flag in ("true", "1", "yes", "y", "bus"):
                    is_bus_servo = True
                elif flag in ("false", "0", "no", "n", "pca", "pwm"):
                    is_bus_servo = False

            if is_bus_servo is None:
                servo_type_val = web_servo.get("servo_type")
                if servo_type_val is None and outer:
                    servo_type_val = outer.get("servo_type")
                if servo_type_val is not None:
                    servo_type_str = str(servo_type_val).strip().lower()
                    if servo_type_str in ("bus", "bus_servo"):
                        is_bus_servo = True
                    elif servo_type_str in ("pca", "pca_servo", "pwm"):
                        is_bus_servo = False

            if is_bus_servo is None:
                return None

            servo_id = web_servo.get("servo_id", web_servo.get("channel"))
            if servo_id is None:
                return None

            position = web_servo.get("position", web_servo.get("angle"))
            if position is None:
                return None

            return {
                "servo_type": "bus" if is_bus_servo else "pca",
                "servo_id": int(servo_id),
                "position": int(position),
                "speed": int(web_servo.get("speed", 100)),
                "port": int(web_servo.get("port", 0)),
            }
        except (TypeError, ValueError):
            LOGGER.exception("解析 web_servo 失败: %s", web_servo)
            return None

    def _parse_full_format(self, data: dict[str, Any]) -> Optional[dict[str, Any]]:
        """解析扁平完整格式。"""
        try:
            servo_type = "bus"
            servo_type_val = data.get("servo_type")
            if servo_type_val is not None:
                servo_type_str = str(servo_type_val).strip().lower()
                if servo_type_str in ("bus", "bus_servo"):
                    servo_type = "bus"
                elif servo_type_str in ("pca", "pca_servo", "pwm"):
                    servo_type = "pca"
            elif "channel" in data and "servo_id" not in data and "id" not in data:
                servo_type = "pca"

            servo_id = data.get("servo_id", data.get("id", data.get("channel")))
            if servo_id is None:
                return None

            raw_position = None
            for key in ("position", "angle", "p", "pulse"):
                if key in data:
                    raw_position = data[key]
                    break
            if raw_position is None:
                return None

            return {
                "servo_type": servo_type,
                "servo_id": int(servo_id),
                "position": int(raw_position),
                "speed": int(data.get("speed", data.get("s", 100))),
                "port": int(data.get("port", 0)),
            }
        except (TypeError, ValueError):
            LOGGER.exception("解析完整舵机格式失败: %s", data)
            return None


@dataclass(slots=True)
class DriveCommand:
    """标准化后的 LX 驱动命令。"""

    servo_id: int
    angle_deg: float
    lx_position: int
    speed_ms: int
    source: str
    raw_position: Any


class LXSerialBus:
    """LX 串口发送器。"""

    def __init__(
        self,
        port: str,
        baud: int,
        timeout_s: float,
        write_timeout_s: float,
        inter_packet_gap_s: float,
        debug: bool,
    ) -> None:
        self.protocol = LXBusServoProtocol()
        self.debug = bool(debug)
        self.inter_packet_gap_s = max(0.0, float(inter_packet_gap_s))
        self._lock = asyncio.Lock()
        self._serial = serial.Serial(
            port=port,
            baudrate=int(baud),
            timeout=float(timeout_s),
            write_timeout=float(write_timeout_s),
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self.port = port

    @staticmethod
    def _hex(data: bytes) -> str:
        return " ".join(f"{byte:02X}" for byte in data)

    async def close(self) -> None:
        """关闭串口。"""
        if self._serial and self._serial.is_open:
            self._serial.close()

    async def _write_packet(self, packet: bytes, desc: str) -> None:
        async with self._lock:
            self._serial.write(packet)
            self._serial.flush()
            if self.debug:
                LOGGER.info("[TX] %s :: %s", desc, self._hex(packet))
            if self.inter_packet_gap_s > 0:
                await asyncio.sleep(self.inter_packet_gap_s)

    async def set_servo_mode(self, servo_id: int) -> None:
        """切回舵机模式。"""
        packet = self.protocol.encode_or_motor_mode_write(
            servo_id=servo_id,
            mode=0,
            drive_mode=0,
            speed=0,
        )
        await self._write_packet(packet, f"mode-servo id={servo_id}")

    async def torque_restore(self, servo_id: int, compat: bool = False) -> None:
        """恢复扭矩。"""
        if compat:
            packet = self.protocol.encode_torque_restore_compat(servo_id=servo_id)
        else:
            packet = self.protocol.encode_torque_restore(servo_id=servo_id)
        await self._write_packet(
            packet,
            f"torque-restore id={servo_id} compat={int(bool(compat))}",
        )

    async def move(self, servo_id: int, lx_position: int, time_ms: int) -> None:
        """发送移动命令。"""
        lx_position = clamp_int(lx_position, 0, 1000)
        time_ms = clamp_int(time_ms, 0, 30000)
        packet = self.protocol.encode_move_command(
            servo_id=servo_id,
            position=lx_position,
            speed=time_ms,
        )
        await self._write_packet(
            packet,
            f"move id={servo_id} pos={lx_position} time_ms={time_ms}",
        )


class LXDriveServer:
    """独立 WebSocket + LX 串口驱动。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.host = args.host
        self.port = int(args.port)
        self.device_id = args.device_id
        self.robot_device_id = args.robot_device_id
        self.debug = bool(args.debug)
        self.allowed_ids = set(parse_servo_ids(args.ids))
        self.message_parser = WSMessageParser(debug=self.debug)
        self.serial_bus = LXSerialBus(
            port=args.serial,
            baud=args.baud,
            timeout_s=args.serial_timeout,
            write_timeout_s=args.serial_write_timeout,
            inter_packet_gap_s=args.inter_packet_gap_ms / 1000.0,
            debug=self.debug,
        )
        self.clients: set[Any] = set()
        self.client_info: dict[Any, dict[str, str]] = {}
        self.bus_state: dict[str, int] = {}
        self.server = None
        self.running = False
        self.heartbeat_task: Optional[asyncio.Task] = None

    async def start(self) -> None:
        """启动 WebSocket 服务。"""
        self.server = await serve(
            self.handle_client,
            self.host,
            self.port,
            ping_interval=10,
            ping_timeout=5,
        )
        self.running = True
        LOGGER.info(
            "WS 驱动启动: ws://%s:%s  serial=%s  ids=%s",
            self.host,
            self.port,
            self.args.serial,
            sorted(self.allowed_ids),
        )

        if self.args.heartbeat_interval > 0:
            self.heartbeat_task = asyncio.create_task(self._heartbeat_loop())

    async def stop(self) -> None:
        """停止服务并清理资源。"""
        self.running = False

        if self.heartbeat_task is not None:
            self.heartbeat_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self.heartbeat_task

        for client in list(self.clients):
            with contextlib.suppress(Exception):
                await client.close()

        if self.server is not None:
            self.server.close()
            await self.server.wait_closed()

        await self.serial_bus.close()
        LOGGER.info("WS 驱动已停止")

    async def initialize_servos(self) -> None:
        """启动时初始化扭矩和模式。"""
        if self.args.no_init_on_start:
            LOGGER.info("已跳过启动初始化")
            return

        LOGGER.info(
            "开始初始化 LX 舵机: ids=%s compat_torque=%s set_servo_mode=%s",
            sorted(self.allowed_ids),
            self.args.compat_torque,
            not self.args.skip_servo_mode_on_start,
        )
        for servo_id in sorted(self.allowed_ids):
            await self.serial_bus.torque_restore(
                servo_id=servo_id,
                compat=self.args.compat_torque,
            )
            if not self.args.skip_servo_mode_on_start:
                await self.serial_bus.set_servo_mode(servo_id=servo_id)
        LOGGER.info("LX 舵机初始化完成")

    async def handle_client(
        self,
        websocket: Any,
        path: Optional[str] = None,
    ) -> None:
        """处理单个客户端连接。"""
        del path
        self.clients.add(websocket)
        LOGGER.info("客户端连接: %s", websocket.remote_address)
        try:
            async for raw_message in websocket:
                await self._process_client_message(websocket, raw_message)
        except ConnectionClosed:
            LOGGER.info("客户端断开: %s", websocket.remote_address)
        except Exception:
            LOGGER.exception("处理客户端消息失败")
        finally:
            self.clients.discard(websocket)
            self.client_info.pop(websocket, None)
            await self._broadcast_user_list()

    async def _process_client_message(
        self,
        websocket: Any,
        raw_message: str,
    ) -> None:
        """解析并处理一条客户端消息。"""
        LOGGER.info(
            "WS原始消息: remote=%s raw=%s",
            websocket.remote_address,
            raw_message,
        )
        data = self.message_parser.parse_message(raw_message)
        if data is None:
            await self._send_json(
                websocket,
                {
                    "type": "error",
                    "status": "error",
                    "device_id": self.device_id,
                    "message": "JSON 解析失败",
                    "timestamp": int(time.time()),
                },
            )
            return

        msg_type = self.message_parser.get_message_type(data)

        if msg_type == "register":
            await self._handle_register(websocket, data)
            return

        if msg_type == "heartbeat":
            await self._send_json(websocket, self._heartbeat_payload())
            return

        if msg_type == "status_query":
            await self._send_json(websocket, self._build_status_response())
            return

        if msg_type == "bvh_play":
            await self._send_json(
                websocket,
                {
                    "type": "error",
                    "status": "error",
                    "device_id": self.device_id,
                    "message": "当前脚本不处理 BVH 动作",
                    "timestamp": int(time.time()),
                },
            )
            return

        commands, ignored = self._extract_drive_commands(data)
        if not commands:
            await self._send_json(
                websocket,
                {
                    "type": "error",
                    "status": "error",
                    "device_id": self.device_id,
                    "message": "未解析到可执行的 LX 总线舵机命令",
                    "ignored_count": ignored,
                    "timestamp": int(time.time()),
                },
            )
            return

        await self._execute_commands(commands)
        await self._send_json(
            websocket,
            self._build_ack_payload(
                original_data=data,
                commands=commands,
                ignored=ignored,
            ),
        )

    async def _execute_commands(self, commands: list[DriveCommand]) -> None:
        """顺序执行一批 LX 命令。"""
        for command in commands:
            LOGGER.info(
                "收到WS总线舵机角度: ID=%s ANGLE=%+.2fdeg -> LX_POS=%s SPEED=%s source=%s raw=%s",
                command.servo_id,
                command.angle_deg,
                command.lx_position,
                command.speed_ms,
                command.source,
                command.raw_position,
            )
            await self.serial_bus.move(
                servo_id=command.servo_id,
                lx_position=command.lx_position,
                time_ms=command.speed_ms,
            )
            self.bus_state[f"id_{command.servo_id}"] = int(round(command.angle_deg))

    def _extract_drive_commands(self, payload: Any) -> tuple[list[DriveCommand], int]:
        """从任意 WS payload 中提取本脚本可执行的 LX 命令。"""
        candidates = list(self._iter_servo_payloads(payload))
        commands: list[DriveCommand] = []
        ignored = 0

        for candidate in candidates:
            command = self._parse_drive_command(candidate)
            if command is None:
                ignored += 1
                continue
            commands.append(command)

        return commands, ignored

    def _iter_servo_payloads(self, payload: Any) -> Iterable[dict[str, Any]]:
        """递归遍历所有可能的舵机命令载荷。"""
        if payload is None:
            return

        if isinstance(payload, str):
            parsed = self.message_parser.parse_message(payload)
            if parsed is not None:
                yield from self._iter_servo_payloads(parsed)
            return

        if isinstance(payload, list):
            for item in payload:
                yield from self._iter_servo_payloads(item)
            return

        if not isinstance(payload, dict):
            return

        if payload.get("type") == "private":
            yield from self._iter_servo_payloads(payload.get("content"))
            return

        content = payload.get("content")
        if isinstance(content, (list, dict, str)) and payload.get("type") in ("broadcast",):
            yield from self._iter_servo_payloads(content)

        if "servo_control" in payload and isinstance(payload["servo_control"], list):
            yield from self._iter_servo_payloads(payload["servo_control"])

        if self._looks_like_servo_command(payload):
            yield payload

    @staticmethod
    def _looks_like_servo_command(payload: dict[str, Any]) -> bool:
        """粗判当前 dict 是否像一条舵机命令。"""
        if "web_servo" in payload:
            return True
        if all(key in payload for key in ("b", "c", "p")):
            return True
        if any(key in payload for key in ("servo_id", "channel", "position", "angle")):
            return True
        return False

    def _parse_drive_command(self, payload: dict[str, Any]) -> Optional[DriveCommand]:
        """把单条 payload 归一化为 LX 指令。"""
        try:
            if all(key in payload for key in ("b", "c", "p")):
                return self._parse_bcp_drive_command(payload)

            parsed = self.message_parser.parse_servo_control(payload)
            if parsed is None:
                return None
            if str(parsed.get("servo_type", "")).lower() != "bus":
                return None

            return self._build_drive_command(
                servo_id=int(parsed["servo_id"]),
                raw_position=parsed.get("position"),
                speed=int(parsed.get("speed", 100)),
                source="parsed",
            )
        except Exception:
            LOGGER.exception("解析舵机命令失败: %s", payload)
            return None

    def _parse_bcp_drive_command(self, payload: dict[str, Any]) -> Optional[DriveCommand]:
        """兼容 {b,c,p,s} 简写协议。"""
        try:
            b_flag = int(payload.get("b", 0))
            if b_flag != -1:
                return None
            return self._build_drive_command(
                servo_id=int(payload["c"]),
                raw_position=payload["p"],
                speed=int(payload.get("s", 100)),
                source="bcp",
            )
        except Exception:
            LOGGER.exception("解析 BCP 舵机命令失败: %s", payload)
            return None

    def _build_drive_command(
        self,
        servo_id: int,
        raw_position: Any,
        speed: int,
        source: str,
    ) -> Optional[DriveCommand]:
        """将 WS 位置值解释为中心角，再映射到 LX 原生位置。"""
        if servo_id not in self.allowed_ids:
            LOGGER.warning("忽略未托管的舵机 ID=%s raw_position=%s", servo_id, raw_position)
            return None

        angle_deg, mode = self._coerce_bus_angle(raw_position)
        speed_ms = clamp_int(speed, 0, 30000)
        lx_position = centered_angle_to_lx_position(angle_deg)
        return DriveCommand(
            servo_id=servo_id,
            angle_deg=angle_deg,
            lx_position=lx_position,
            speed_ms=speed_ms,
            source=f"{source}/{mode}",
            raw_position=raw_position,
        )

    def _coerce_bus_angle(self, raw_position: Any) -> tuple[float, str]:
        """将输入位置解释为中心角。

        默认模式:
        - [-90, 90]: 直接视为中心角
        - [500, 2500]: 视为统一脉宽
        - [125, 875]: 若开启 --accept-lx-native，则视为 LX 原生位置

        注意:
        0~180 与当前项目里的中心角协议存在歧义，因此默认不自动兼容。
        仅在显式传入 --legacy-0-180 时，才把它解释为旧的中间角语义。
        """
        value = float(raw_position)

        if -90.0 <= value <= 90.0:
            return value, "centered_deg"

        if 500.0 <= value <= 2500.0:
            return pulse_to_centered_angle(value), "pulse_us"

        if self.args.accept_lx_native and 125.0 <= value <= 875.0:
            return lx_position_to_centered_angle(int(round(value))), "lx_native"

        if self.args.legacy_0_180 and 0.0 <= value <= 180.0:
            return 90.0 - value, "legacy_0_180"

        raise ValueError(
            f"不支持的位置值 {raw_position}。"
            "默认只接受中心角[-90,90]；"
            "如需兼容旧 0~180 语义，请加 --legacy-0-180。"
        )

    async def _handle_register(
        self,
        websocket: Any,
        data: dict[str, Any],
    ) -> None:
        """兼容现有 register 协议。"""
        client_name = data.get("name", f"client_{len(self.client_info) + 1}")
        client_id = str(uuid.uuid4())
        self.client_info[websocket] = {"id": client_id, "name": client_name}

        await self._send_json(
            websocket,
            {
                "type": "connected",
                "clientName": client_name,
                "onlineClients": self._get_online_users(),
                "device_id": self.device_id,
                "timestamp": int(time.time()),
            },
        )
        LOGGER.info("客户端已注册: %s (%s)", client_name, client_id)
        await self._broadcast_user_list()

    def _get_online_users(self) -> list[dict[str, str]]:
        """返回在线用户列表，保留虚拟机器人设备。"""
        users = [{"id": self.robot_device_id, "name": self.device_id}]
        users.extend(
            {"id": info["id"], "name": info["name"]}
            for info in self.client_info.values()
        )
        return users

    async def _broadcast_user_list(self) -> None:
        """向所有在线客户端广播在线用户列表。"""
        if not self.clients:
            return
        payload = {
            "type": "broadcast",
            "content": "用户列表更新",
            "onlineUsers": self._get_online_users(),
            "timestamp": int(time.time()),
        }
        await self._broadcast_json(payload)

    def _heartbeat_payload(self) -> dict[str, Any]:
        """生成心跳响应。"""
        return {
            "type": "heartbeat",
            "status": "online",
            "device_id": self.device_id,
            "timestamp": int(time.time()),
        }

    async def _heartbeat_loop(self) -> None:
        """定时广播心跳。"""
        while self.running:
            await asyncio.sleep(self.args.heartbeat_interval)
            if not self.clients:
                continue
            await self._broadcast_json(self._heartbeat_payload())

    def _build_status_response(self) -> dict[str, Any]:
        """返回当前缓存状态。

        这里返回的是最近一次命令角度，不是串口读回值。
        """
        return {
            "character_name": self.device_id,
            "bus_servos": dict(self.bus_state),
            "pwm_servos": {},
            "time": time.strftime("%Y-%m-%d-%H:%M:%S"),
            "current_status": {
                "movement_active": False,
                "listening": False,
                "action": "idle",
                "led_state": "off",
            },
            "result_code": 200,
            "timestamp": int(time.time()),
        }

    def _build_ack_payload(
        self,
        original_data: dict[str, Any],
        commands: list[DriveCommand],
        ignored: int,
    ) -> dict[str, Any]:
        """按现有协议返回 ack。"""
        command_items = [
            {
                "servo_id": command.servo_id,
                "angle": round(command.angle_deg, 2),
                "lx_position": command.lx_position,
                "speed": command.speed_ms,
                "source": command.source,
            }
            for command in commands
        ]
        if original_data.get("type") == "private":
            return {
                "type": "private_ack",
                "status": "processed",
                "device_id": self.device_id,
                "message": "舵机命令已发送",
                "accepted_count": len(commands),
                "ignored_count": ignored,
                "commands": command_items,
                "timestamp": int(time.time()),
            }

        return {
            "type": "servo_control_ack",
            "status": "accepted",
            "device_id": self.device_id,
            "data": {
                "accepted_count": len(commands),
                "ignored_count": ignored,
                "commands": command_items,
            },
            "timestamp": int(time.time()),
        }

    async def _send_json(
        self,
        websocket: Any,
        payload: dict[str, Any],
    ) -> None:
        """向单个客户端发送 JSON。"""
        await websocket.send(json.dumps(payload, ensure_ascii=False))

    async def _broadcast_json(self, payload: dict[str, Any]) -> None:
        """向所有客户端广播 JSON。"""
        if not self.clients:
            return
        raw = json.dumps(payload, ensure_ascii=False)
        await asyncio.gather(
            *(client.send(raw) for client in list(self.clients)),
            return_exceptions=True,
        )


def build_parser() -> argparse.ArgumentParser:
    """构建命令行参数。"""
    parser = argparse.ArgumentParser(
        description="独立 WebSocket -> LX 舵机驱动（绕过 ROS）",
    )
    parser.add_argument("--host", default="0.0.0.0", help="WS 监听地址")
    parser.add_argument("--port", type=int, default=9105, help="WS 监听端口")
    parser.add_argument("--device-id", default="robot", help="设备 ID")
    parser.add_argument(
        "--robot-device-id",
        default=ROBOT_DEVICE_ID,
        help="对外暴露的机器人虚拟设备 ID",
    )
    parser.add_argument(
        "--serial",
        default="/dev/ttyAMA1",
        help="LX 串口设备，默认 /dev/ttyAMA1",
    )
    parser.add_argument("--baud", type=int, default=115200, help="串口波特率")
    parser.add_argument(
        "--ids",
        default=DEFAULT_IDS,
        help="托管的 LX 舵机 ID，示例 21-37 或 21-37,40",
    )
    parser.add_argument(
        "--serial-timeout",
        type=float,
        default=0.15,
        help="串口读超时(秒)，默认 0.15",
    )
    parser.add_argument(
        "--serial-write-timeout",
        type=float,
        default=0.2,
        help="串口写超时(秒)，默认 0.2",
    )
    parser.add_argument(
        "--inter-packet-gap-ms",
        type=float,
        default=8.0,
        help="串口帧间隔(ms)，默认 8",
    )
    parser.add_argument(
        "--heartbeat-interval",
        type=float,
        default=15.0,
        help="广播心跳间隔(秒)，设为 0 可关闭",
    )
    parser.add_argument(
        "--compat-torque",
        action="store_true",
        help="启动初始化时使用兼容扭矩指令 0x1E",
    )
    parser.add_argument(
        "--no-init-on-start",
        action="store_true",
        help="启动时不批量发送 torque_restore / mode_servo",
    )
    parser.add_argument(
        "--skip-servo-mode-on-start",
        action="store_true",
        help="启动时只恢复扭矩，不强制切回舵机模式",
    )
    parser.add_argument(
        "--legacy-0-180",
        action="store_true",
        help="兼容旧的 0~180 中间角语义，按 centered = 90 - value 解释",
    )
    parser.add_argument(
        "--accept-lx-native",
        action="store_true",
        help="允许直接把 125~875 解释为 LX 原生位置值",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="打印更详细的 WS 与串口日志",
    )
    return parser


async def _run(args: argparse.Namespace) -> None:
    """运行主逻辑。"""
    server = LXDriveServer(args)
    stop_event = asyncio.Event()

    def _request_stop() -> None:
        if not stop_event.is_set():
            LOGGER.info("收到退出信号，准备停止...")
            stop_event.set()

    loop = asyncio.get_running_loop()
    for sig_name in ("SIGINT", "SIGTERM"):
        if hasattr(signal, sig_name):
            with contextlib.suppress(NotImplementedError):
                loop.add_signal_handler(getattr(signal, sig_name), _request_stop)

    await server.start()
    try:
        await server.initialize_servos()
        await stop_event.wait()
    finally:
        await server.stop()


def main() -> int:
    """命令行入口。"""
    parser = build_parser()
    args = parser.parse_args()
    setup_logging(args.debug)
    try:
        asyncio.run(_run(args))
    except KeyboardInterrupt:
        LOGGER.info("收到 Ctrl+C，退出")
    except Exception:
        LOGGER.exception("驱动启动失败")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
