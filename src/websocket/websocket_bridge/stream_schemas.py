"""
标准 JSON 流格式定义和验证
"""

import json
from typing import Dict, Any, Optional
from pathlib import Path


class StreamSchemas:
    """JSON 流格式管理类"""
    
    def __init__(self, config_dir: Optional[str] = None):
        """
        初始化 Stream Schemas
        
        Args:
            config_dir: 配置文件目录，如果为 None，则使用包内默认配置
        """
        self.config_dir = config_dir or str(Path(__file__).parent.parent / "config")
        self.web2jiyuan_schema = self._load_schema("std_web2ros_stream.json")
        self.jiyuan2web_schema = self._load_schema("std_ros2web_stream.json")
    
    def _load_schema(self, filename: str) -> Dict[str, Any]:
        """加载 JSON schema 文件"""
        schema_path = Path(self.config_dir) / filename
        try:
            with open(schema_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            print(f"[StreamSchemas] 加载 {filename} 失败: {e}")
            return {}
    
    def get_web2jiyuan_schema(self) -> Dict[str, Any]:
        """获取 Web → Jiyuan 的 JSON schema"""
        return self.web2jiyuan_schema
    
    def get_jiyuan2web_schema(self) -> Dict[str, Any]:
        """获取 Jiyuan → Web 的 JSON schema"""
        return self.jiyuan2web_schema
    
    def validate_web_command(self, data: Dict[str, Any]) -> bool:
        """
        验证来自 Web 的命令格式
        
        Args:
            data: 接收到的 JSON 对象
            
        Returns:
            bool: 是否符合格式
        """
        # 检查关键字段
        if not isinstance(data, dict):
            return False

        # 新格式: 标准 web_servo payload
        if "web_servo" in data:
            return True

        # 如果包含 {b, c, p} 字段，则为简写协议
        if 'b' in data and 'c' in data and 'p' in data:
            return True
        
        # 否则检查其他舵机控制字段
        if any(key in data for key in ['id', 'servo_id', 'channel', 'c', 'angle', 'position', 'p']):
            return True
        
        return False
    
    def validate_jiyuan_response(self, data: Dict[str, Any]) -> bool:
        """
        验证来自 Jiyuan 的响应格式
        
        Args:
            data: 要验证的 JSON 对象
            
        Returns:
            bool: 是否符合格式
        """
        if not isinstance(data, dict):
            return False
        
        # 检查必要字段
        required_fields = ['character_name', 'current_status']
        for field in required_fields:
            if field not in data:
                return False
        
        return True
    
    def create_servo_command(self, servo_type: str, servo_id: int, 
                           position: int, speed: int = 100, 
                           port: int = 0) -> Dict[str, Any]:
        """
        创建标准的舵机控制命令
        
        Args:
            servo_type: "bus" 或 "pca"
            servo_id: 舵机 ID 或通道号
            position: 位置 (0-180 度或 0-4095 tick)
            speed: 速度 (1-255, 仅总线舵机)
            port: 总线舵机端口 (0-based)
            
        Returns:
            dict: 标准命令格式
        """
        b_flag = -1 if servo_type == "bus" else 0
        
        return {
            "b": b_flag,
            "c": servo_id,
            "p": position,
            "s": speed,
            "port": port
        }
    
    def create_jiyuan_response(self, status_code: int, message: str = "",
                             servo_states: Optional[Dict] = None,
                             system_info: Optional[Dict] = None) -> Dict[str, Any]:
        """
        创建标准的机器人响应（遵循 std_jiyuan2web_stream.json）

        Args:
            status_code: 状态码 (200 为成功)
            message: 响应消息
            servo_states: 舵机状态字典
            system_info: 系统信息字典

        Returns:
            dict: 标准响应格式
        """
        import time

        response = {
            "character_name": "robot",
            "bus_servos": servo_states.get("bus_servos", {}) if servo_states else {},
            "pwm_servos": servo_states.get("pwm_servos", {}) if servo_states else {},
            "time": time.strftime("%Y-%m-%d-%H:%M:%S"),
            "current_status": {
                "movement_active": False,
                "listening": False,
                "action": "idle",
                "led_state": "off"
            },
            "result_code": status_code
        }

        if system_info:
            response.update(system_info)

        return response


# 全局实例
_schemas = None


def get_stream_schemas(config_dir: Optional[str] = None) -> StreamSchemas:
    """获取全局 StreamSchemas 实例"""
    global _schemas
    if _schemas is None:
        _schemas = StreamSchemas(config_dir)
    return _schemas
