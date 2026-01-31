"""
WebSocket 处理器 - 与 ROS 2 集成
"""

import json
import time
import asyncio
from typing import Optional, Dict, Any, Callable
from .message_handler import MessageHandler, MessageType
from .error_codes import (
    ErrorCode,
    ErrorResponse,
    SuccessResponse,
    InvalidServoCommandException,
    ServoCommandFailedException
)
from .logger import get_logger

logger = get_logger(__name__)


class WebSocketHandler:
    """
    WebSocket 通信处理器
    
    1. 维护 WebSocket 连接
    2. 解析来自 Web 的消息
    3. 转发到 ROS 2 话题
    4. 订阅 ROS 2 状态并回传 Web
    """
    
    def __init__(self, device_id: str = "default", debug: bool = False):
        """
        初始化 WebSocket 处理器

        Args:
            device_id: 设备 ID
            debug: 是否启用调试日志
        """
        self.device_id = device_id
        self.debug = debug
        self.message_handler = MessageHandler(debug=debug)
        
        # ROS 2 相关的回调
        self.on_servo_command = None  # Callable[[dict], None]
        self.on_heartbeat = None  # Callable[[], None]
        self.on_status_query = None  # Callable[[], None]
        
        # 状态管理
        self.servo_state = {}
        self.system_state = {
            "cpu_temp": 0,
            "free_memory": 0,
            "uptime": 0
        }
        self.last_heartbeat = time.time()
    
    def register_servo_command_handler(self, callback: Callable):
        """
        注册舵机命令回调
        
        Args:
            callback: async def callback(servo_cmd: dict) -> None
        """
        self.on_servo_command = callback
    
    def register_heartbeat_handler(self, callback: Callable):
        """
        注册心跳处理回调
        
        Args:
            callback: async def callback() -> None
        """
        self.on_heartbeat = callback
    
    def register_status_query_handler(self, callback: Callable):
        """
        注册状态查询回调
        
        Args:
            callback: async def callback() -> dict
        """
        self.on_status_query = callback
    
    def register_message_handler(self, msg_type: str, callback: Callable):
        """
        为特定消息类型注册处理器
        
        Args:
            msg_type: 消息类型 (如 "servo_control", "heartbeat" 等)
            callback: async def callback(data: dict) -> Optional[str]
        """
        self.message_handler.register_handler(msg_type, callback)
    
    async def handle_message(self, raw_message: str) -> Optional[str]:
        """
        处理接收到的 WebSocket 消息
        
        Args:
            raw_message: 原始 JSON 字符串
            
        Returns:
            str: 响应消息，或 None
        """
        # 解析消息
        data = self.message_handler.parse_message(raw_message)
        if data is None:
            return None
        
        # 获取消息类型
        msg_type = self.message_handler.get_message_type(data)
        
        if self.debug:
            print(f"[WebSocketHandler] 处理消息: {msg_type.value}")
        
        # 根据消息类型分发
        if msg_type == MessageType.HEARTBEAT:
            return await self._handle_heartbeat(data)
        elif msg_type == MessageType.SERVO_CONTROL:
            return await self._handle_servo_control(data)
        elif msg_type == MessageType.STATUS_QUERY:
            return await self._handle_status_query(data)
        elif msg_type == MessageType.REGISTER:
            return await self._handle_register(data)
        elif msg_type == MessageType.BROADCAST:
            return await self._handle_broadcast(data)
        elif msg_type == MessageType.PRIVATE:
            return await self._handle_private(data)
        else:
            return None
    
    async def _handle_heartbeat(self, data: Dict[str, Any]) -> Optional[str]:
        """处理心跳消息"""
        self.last_heartbeat = time.time()
        
        if self.on_heartbeat:
            try:
                await self.on_heartbeat()
            except Exception as e:
                print(f"[WebSocketHandler] 心跳回调失败: {e}")
        
        response = {
            "type": "heartbeat",
            "status": "online",
            "device_id": self.device_id,
            "timestamp": int(time.time())
        }
        
        if self.debug:
            print(f"[WebSocketHandler] 发送心跳响应")
        
        return json.dumps(response, ensure_ascii=False)
    
    async def _handle_servo_control(self, data: Dict[str, Any]) -> Optional[str]:
        """处理舵机控制命令"""
        servo_cmd = self.message_handler.parse_servo_control(data)

        if servo_cmd is None:
            print(f"[WebSocketHandler] 无法解析舵机控制命令: {data}")
            return ErrorResponse.create(
                error_code=ErrorCode.INVALID_SERVO_COMMAND,
                message="舵机控制命令格式无效",
                details={"received_data": data},
                device_id=self.device_id
            )

        if self.debug:
            print(f"[WebSocketHandler] 解析舵机命令: {servo_cmd}")

        # 调用 ROS 2 命令处理器
        if self.on_servo_command:
            try:
                await self.on_servo_command(servo_cmd)
            except Exception as e:
                print(f"[WebSocketHandler] 舵机命令处理失败: {e}")
                return ErrorResponse.create(
                    error_code=ErrorCode.SERVO_COMMAND_FAILED,
                    message=f"舵机命令执行失败: {str(e)}",
                    details={"command": servo_cmd, "exception": str(e)},
                    device_id=self.device_id
                )

        # 返回确认响应
        return SuccessResponse.create(
            response_type="servo_control_ack",
            data={
                "status": "accepted",
                "command": servo_cmd
            },
            device_id=self.device_id
        )
    
    async def _handle_status_query(self, data: Dict[str, Any]) -> Optional[str]:
        """处理状态查询"""
        if self.on_status_query:
            try:
                status = await self.on_status_query()
                if status:
                    return json.dumps(status, ensure_ascii=False)
            except Exception as e:
                print(f"[WebSocketHandler] 状态查询失败: {e}")
        
        return self._create_status_response()
    
    async def _handle_register(self, data: Dict[str, Any]) -> Optional[str]:
        """处理设备注册"""
        device_name = data.get("name", self.device_id)
        
        response = {
            "type": "register_ack",
            "status": "success",
            "device_id": self.device_id,
            "device_name": device_name,
            "supported_commands": self._get_supported_commands(),
            "timestamp": int(time.time())
        }
        
        if self.debug:
            print(f"[WebSocketHandler] 设备已注册: {device_name}")
        
        return json.dumps(response, ensure_ascii=False)
    
    async def _handle_broadcast(self, data: Dict[str, Any]) -> Optional[str]:
        """处理广播消息"""
        content = data.get("content", {})
        
        if self.debug:
            print(f"[WebSocketHandler] 收到广播: {content}")
        
        # 如果广播内容包含舵机命令
        servo_cmd = self.message_handler.parse_servo_control(content)
        if servo_cmd and self.on_servo_command:
            try:
                await self.on_servo_command(servo_cmd)
            except Exception as e:
                print(f"[WebSocketHandler] 广播舵机命令处理失败: {e}")
        
        return None
    
    async def _handle_private(self, data: Dict[str, Any]) -> Optional[str]:
        """处理私有消息"""
        content = data.get("content", data)
        
        if self.debug:
            print(f"[WebSocketHandler] 收到私有消息")
        
        # 尝试解析为舵机命令
        servo_cmd = self.message_handler.parse_servo_control(content)
        if servo_cmd and self.on_servo_command:
            try:
                await self.on_servo_command(servo_cmd)
            except Exception as e:
                print(f"[WebSocketHandler] 私有消息舵机命令处理失败: {e}")
            
            response = {
                "type": "private_ack",
                "status": "processed",
                "device_id": self.device_id,
                "timestamp": int(time.time())
            }
            return json.dumps(response, ensure_ascii=False)
        
        return None
    
    def _create_status_response(self) -> str:
        """创建状态响应 - 使用标准格式"""
        import time

        response = {
            "character_name": "robot",
            "bus_servos": self.servo_state.get("bus_servos", {}),
            "pwm_servos": self.servo_state.get("pwm_servos", {}),
            "time": time.strftime("%Y-%m-%d-%H:%M:%S"),
            "current_status": {
                "movement_active": False,
                "listening": False,
                "action": "idle",
                "led_state": "off"
            },
            "result_code": 200,
            "timestamp": int(time.time())
        }
        return json.dumps(response, ensure_ascii=False)
    
    def _get_supported_commands(self) -> Dict[str, Any]:
        """获取支持的命令列表"""
        return {
            "servo_control": [
                "{b: -1, c: servo_id, p: angle_0_180, s: speed}",  # 总线舵机
                "{b: 0, c: channel_0_15, p: pwm_value}",  # PCA 舵机
                "{servo_type: 'bus', servo_id: 1, position: 90, speed: 100}",
                "{servo_type: 'pca', servo_id: 0, position: 90}",
                "{character_name: 'robot', web_servo: {is_bus_servo: true, servo_id: 1, position: 90, speed: 100}}"
            ],
            "heartbeat": ["keep connection alive"],
            "status_query": ["request device status"]
        }
    
    def update_servo_state(self, servo_id: int, servo_type: str, position: int):
        """
        更新舵机状态
        
        Args:
            servo_id: 舵机 ID
            servo_type: 舵机类型 ("bus" 或 "pca")
            position: 当前位置
        """
        if servo_type == "bus":
            if "bus_servos" not in self.servo_state:
                self.servo_state["bus_servos"] = {}
            self.servo_state["bus_servos"][f"id_{servo_id}"] = position
        elif servo_type == "pca":
            if "pwm_servos" not in self.servo_state:
                self.servo_state["pwm_servos"] = {}
            self.servo_state["pwm_servos"][f"id_{servo_id}"] = position
    
    def update_system_state(self, **kwargs):
        """
        更新系统状态
        
        Args:
            cpu_temp: CPU 温度
            free_memory: 空闲内存
            uptime: 运行时间
        """
        self.system_state.update(kwargs)
    
    def get_uptime(self) -> float:
        """获取从最后一次心跳后经过的时间"""
        return time.time() - self.last_heartbeat
