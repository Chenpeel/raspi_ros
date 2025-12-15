"""
WebSocket 消息处理器 - 路由和转换消息
"""

import json
import time
from typing import Dict, Any, Optional, Callable
from enum import Enum
from .logger import get_logger

logger = get_logger(__name__)


class MessageType(Enum):
    """消息类型定义"""
    HEARTBEAT = "heartbeat"
    SERVO_CONTROL = "servo_control"
    BROADCAST = "broadcast"
    PRIVATE = "private"
    REGISTER = "register"
    REGISTER_ACK = "register_ack"
    STATUS_QUERY = "status_query"
    UNKNOWN = "unknown"


class MessageHandler:
    """WebSocket 消息处理器"""

    def __init__(self, debug: bool = False):
        """
        初始化消息处理器

        Args:
            debug: 是否启用调试日志
        """
        self.debug = debug
        self.command_handlers = {}  # 命令类型 -> 回调函数映射
        self.message_callbacks = {}  # 消息类型 -> 回调函数列表

        # 配置常量
        self.MAX_BUS_SERVO_ID = 200
        self.BUS_MIN_US = 500
        self.BUS_MAX_US = 2500
        self.PCA_MIN_US = 500
        self.PCA_MAX_US = 2500
        self.MIN_PWM = 110
        self.MAX_PWM = 520
        self.PWM_FREQ = 50

    def register_handler(self, msg_type: str, callback: Callable):
        """
        注册消息处理回调

        Args:
            msg_type: 消息类型
            callback: 回调函数 async def callback(data: dict) -> Optional[str]
        """
        if msg_type not in self.message_callbacks:
            self.message_callbacks[msg_type] = []
        self.message_callbacks[msg_type].append(callback)
        if self.debug:
            logger.debug(f"注册处理器: {msg_type}")

    def parse_message(self, raw_message: str) -> Optional[Dict[str, Any]]:
        """
        解析原始消息

        Args:
            raw_message: 原始 JSON 字符串

        Returns:
            dict: 解析后的消息对象，或 None
        """
        try:
            data = json.loads(raw_message)
            if self.debug:
                logger.debug(f"解析消息成功: {data}")
            return data
        except json.JSONDecodeError as e:
            logger.error(f"JSON 解析失败: {e}")
            return None

    def get_message_type(self, data: Dict[str, Any]) -> MessageType:
        """
        从消息对象中提取消息类型

        Args:
            data: 消息对象

        Returns:
            MessageType: 消息类型
        """
        if not isinstance(data, dict):
            return MessageType.UNKNOWN

        # 尝试多个可能的 type 字段名
        msg_type = (data.get("type") or
                    data.get("Type") or
                    data.get("TYPE") or
                    "").lower().strip()

        if not msg_type:
            # 尝试根据内容推断类型
            if self._is_servo_control(data):
                return MessageType.SERVO_CONTROL
            elif "timestamp" in data and "status" in data:
                return MessageType.HEARTBEAT
            return MessageType.UNKNOWN

        # 映射到枚举
        try:
            return MessageType(msg_type)
        except ValueError:
            return MessageType.UNKNOWN

    def _is_servo_control(self, data: Dict[str, Any]) -> bool:
        """检查是否为舵机控制命令"""
        servo_keys = {'b', 'c', 'p', 'id', 'servo_id', 'channel',
                      'angle', 'position', 'speed'}
        return bool(set(data.keys()) & servo_keys)

    async def process_message(self, data: Dict[str, Any]) -> Optional[str]:
        """
        处理接收到的消息

        Args:
            data: 解析后的消息对象

        Returns:
            str: 响应消息（JSON 字符串），或 None
        """
        msg_type = self.get_message_type(data)

        if self.debug:
            logger.debug(f"消息类型: {msg_type.value}")

        # 调用对应的处理器
        if msg_type in self.message_callbacks:
            for callback in self.message_callbacks[msg_type]:
                response = await callback(data)
                if response:
                    return response

        # 默认处理器
        return await self._default_handler(data, msg_type)

    async def _default_handler(self, data: Dict[str, Any],
                               msg_type: MessageType) -> Optional[str]:
        """默认消息处理器"""
        if msg_type == MessageType.HEARTBEAT:
            return self._create_heartbeat_response()
        elif msg_type == MessageType.STATUS_QUERY:
            return self._create_status_response()
        else:
            if self.debug:
                logger.debug(f"未知消息类型: {msg_type.value}")
            return None

    def _create_heartbeat_response(self) -> str:
        """创建心跳响应"""
        response = {
            "type": "heartbeat",
            "status": "online",
            "device_id": "default",
            "timestamp": int(time.time())
        }
        return json.dumps(response, ensure_ascii=False)

    def _create_status_response(self) -> str:
        """创建状态查询响应"""
        response = {
            "type": "status",
            "character_name": "robot",
            "current_status": {
                "movement_active": False,
                "listening": False,
                "action": "idle",
                "led_state": "off"
            },
            "timestamp": int(time.time()),
            "result_code": 200
        }
        return json.dumps(response, ensure_ascii=False)

    def parse_servo_control(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        解析舵机控制命令（支持多种格式）

        格式解析:
        {
          "b": -1|0,           // -1=总线舵机, 0=PCA舵机
          "c": servo_id,       // 舵机 ID 或通道
          "p": position,       // 位置或角度
          "s": speed           // 可选，速度 (1-255)
        }

        Args:
            data: 消息对象

        Returns:
            dict: 标准化的舵机控制命令，或 None

        标准化格式:
            {
                "servo_type": "bus" | "pca",
                "servo_id": int,
                "position": int,
                "speed": int (可选，总线舵机),
                "port": int (可选，总线舵机)
            }
        """
        if not isinstance(data, dict):
            return None

        # 尝试解析格式 {b, c, p, s}
        if 'b' in data and 'c' in data and 'p' in data:
            return self._parse_bcp_protocol(data)

        # 尝试解析完整格式
        return self._parse_full_format(data)

    def _parse_bcp_protocol(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        解析 {b, c, p} 简写协议

        b == -1: 总线舵机, c 为 ID, p 为微秒 (500-2500)
        b == 0:  PCA 通道, c 为通道 (0-15), p 为 tick
        """
        try:
            b_flag = int(data.get('b', 0))
            c_val = int(data.get('c', 0))
            p_val = int(data.get('p', 0))
            s_val = int(data.get('s', 100))  # 速度
            port_val = int(data.get('port', 0))  # 端口

            if b_flag == -1:
                # 总线舵机
                # 将微秒转换为角度 (0-180)
                position = self._us_to_angle(p_val)
                return {
                    "servo_type": "bus",
                    "servo_id": c_val,
                    "position": position,
                    "speed": s_val,
                    "port": port_val
                }
            elif b_flag == 0:
                # PCA 舵机
                return {
                    "servo_type": "pca",
                    "servo_id": c_val,
                    "position": p_val,
                    "port": 0
                }
        except (ValueError, TypeError) as e:
            logger.error(f"解析 BCP 协议失败: {e}")

        return None

    def _parse_full_format(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """解析完整格式的舵机控制命令"""
        try:
            # 尝试识别舵机类型和 ID
            servo_type = "bus"  # 默认

            # 检查是否为总线舵机
            if 'b' in data:
                b_flag = int(data['b'])
                servo_type = "bus" if b_flag == -1 else "pca"
            elif any(key in data for key in ['servo_id', 'id', 'bus_servo']):
                servo_type = "bus"
            elif 'channel' in data or 'c' in data:
                servo_type = "pca"

            # 获取舵机 ID/通道
            servo_id = None
            for key in ['id', 'servo_id', 'c', 'channel']:
                if key in data:
                    servo_id = int(data[key])
                    break

            if servo_id is None:
                return None

            # 获取位置信息
            position = None
            for key in ['position', 'angle', 'p', 'pulse']:
                if key in data:
                    position = int(data[key])
                    break

            if position is None:
                return None

            return {
                "servo_type": servo_type,
                "servo_id": servo_id,
                "position": position,
                "speed": int(data.get('speed', 100)),
                "port": int(data.get('port', 0))
            }

        except (ValueError, TypeError) as e:
            logger.error(f"解析完整格式失败: {e}")

        return None

    def _us_to_angle(self, us: int) -> int:
        """将微秒转换为角度 (0-180)"""
        try:
            angle = int(round((us - self.BUS_MIN_US) * 180.0 /
                              max(1, (self.BUS_MAX_US - self.BUS_MIN_US))))
        except:
            angle = 90
        return max(0, min(180, angle))

    def _angle_to_us(self, angle: int) -> int:
        """将角度转换为微秒"""
        try:
            us = int(self.BUS_MIN_US + (angle / 180.0) *
                     (self.BUS_MAX_US - self.BUS_MIN_US))
        except:
            us = 1500
        return max(self.BUS_MIN_US, min(self.BUS_MAX_US, us))
