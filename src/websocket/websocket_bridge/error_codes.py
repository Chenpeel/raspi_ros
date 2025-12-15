"""
错误码定义和错误处理工具
"""

from enum import IntEnum
from typing import Dict, Any, Optional
import json
import time


class ErrorCode(IntEnum):
    """WebSocket 错误码枚举"""

    # 成功状态码 (2xx)
    SUCCESS = 200
    ACCEPTED = 202

    # 客户端错误 (4xx)
    BAD_REQUEST = 400
    INVALID_MESSAGE_FORMAT = 4001
    INVALID_SERVO_COMMAND = 4002
    MISSING_REQUIRED_FIELD = 4003
    INVALID_PARAMETER_VALUE = 4004
    UNSUPPORTED_MESSAGE_TYPE = 4005

    # 服务器错误 (5xx)
    INTERNAL_ERROR = 500
    SERVO_COMMAND_FAILED = 5001
    ROS_CALLBACK_FAILED = 5002
    STATUS_QUERY_FAILED = 5003
    HEARTBEAT_FAILED = 5004

    # 连接错误 (6xx)
    CONNECTION_TIMEOUT = 6001
    CONNECTION_CLOSED = 6002
    AUTHENTICATION_FAILED = 6003


class ErrorResponse:
    """错误响应构造器"""

    @staticmethod
    def create(
        error_code: ErrorCode,
        message: str = "",
        details: Optional[Dict[str, Any]] = None,
        device_id: str = "unknown"
    ) -> str:
        """
        创建标准错误响应

        Args:
            error_code: 错误码
            message: 错误消息描述
            details: 详细错误信息（可选）
            device_id: 设备ID

        Returns:
            str: JSON格式的错误响应
        """
        response = {
            "type": "error",
            "status": "error",
            "device_id": device_id,
            "error_code": int(error_code),
            "error_name": error_code.name,
            "message": message or ErrorResponse.get_default_message(error_code),
            "timestamp": int(time.time())
        }

        if details:
            response["details"] = details

        return json.dumps(response, ensure_ascii=False)

    @staticmethod
    def get_default_message(error_code: ErrorCode) -> str:
        """
        获取错误码的默认消息

        Args:
            error_code: 错误码

        Returns:
            str: 默认错误消息
        """
        messages = {
            ErrorCode.SUCCESS: "成功",
            ErrorCode.ACCEPTED: "已接受",
            ErrorCode.BAD_REQUEST: "错误的请求",
            ErrorCode.INVALID_MESSAGE_FORMAT: "消息格式无效",
            ErrorCode.INVALID_SERVO_COMMAND: "舵机控制命令无效",
            ErrorCode.MISSING_REQUIRED_FIELD: "缺少必需字段",
            ErrorCode.INVALID_PARAMETER_VALUE: "参数值无效",
            ErrorCode.UNSUPPORTED_MESSAGE_TYPE: "不支持的消息类型",
            ErrorCode.INTERNAL_ERROR: "内部错误",
            ErrorCode.SERVO_COMMAND_FAILED: "舵机控制命令执行失败",
            ErrorCode.ROS_CALLBACK_FAILED: "ROS回调失败",
            ErrorCode.STATUS_QUERY_FAILED: "状态查询失败",
            ErrorCode.HEARTBEAT_FAILED: "心跳处理失败",
            ErrorCode.CONNECTION_TIMEOUT: "连接超时",
            ErrorCode.CONNECTION_CLOSED: "连接已关闭",
            ErrorCode.AUTHENTICATION_FAILED: "认证失败"
        }

        return messages.get(error_code, "未知错误")

    @staticmethod
    def is_client_error(error_code: ErrorCode) -> bool:
        """判断是否为客户端错误（4xx）"""
        return 4000 <= int(error_code) < 5000

    @staticmethod
    def is_server_error(error_code: ErrorCode) -> bool:
        """判断是否为服务器错误（5xx）"""
        return 5000 <= int(error_code) < 6000

    @staticmethod
    def is_connection_error(error_code: ErrorCode) -> bool:
        """判断是否为连接错误（6xx）"""
        return 6000 <= int(error_code) < 7000


class SuccessResponse:
    """成功响应构造器"""

    @staticmethod
    def create(
        response_type: str,
        data: Optional[Dict[str, Any]] = None,
        device_id: str = "unknown"
    ) -> str:
        """
        创建标准成功响应

        Args:
            response_type: 响应类型（如 "heartbeat", "servo_control_ack"）
            data: 响应数据（可选）
            device_id: 设备ID

        Returns:
            str: JSON格式的成功响应
        """
        response = {
            "type": response_type,
            "status": "success",
            "device_id": device_id,
            "timestamp": int(time.time())
        }

        if data:
            response.update(data)

        return json.dumps(response, ensure_ascii=False)


class WebSocketException(Exception):
    """WebSocket异常基类"""

    def __init__(
        self,
        error_code: ErrorCode,
        message: str = "",
        details: Optional[Dict[str, Any]] = None
    ):
        """
        初始化异常

        Args:
            error_code: 错误码
            message: 错误消息
            details: 详细错误信息
        """
        self.error_code = error_code
        self.message = message or ErrorResponse.get_default_message(error_code)
        self.details = details or {}
        super().__init__(self.message)

    def to_response(self, device_id: str = "unknown") -> str:
        """
        转换为错误响应

        Args:
            device_id: 设备ID

        Returns:
            str: JSON格式的错误响应
        """
        return ErrorResponse.create(
            error_code=self.error_code,
            message=self.message,
            details=self.details,
            device_id=device_id
        )


class InvalidMessageException(WebSocketException):
    """无效消息异常"""

    def __init__(self, message: str = "", details: Optional[Dict[str, Any]] = None):
        super().__init__(ErrorCode.INVALID_MESSAGE_FORMAT, message, details)


class InvalidServoCommandException(WebSocketException):
    """无效舵机命令异常"""

    def __init__(self, message: str = "", details: Optional[Dict[str, Any]] = None):
        super().__init__(ErrorCode.INVALID_SERVO_COMMAND, message, details)


class ServoCommandFailedException(WebSocketException):
    """舵机命令执行失败异常"""

    def __init__(self, message: str = "", details: Optional[Dict[str, Any]] = None):
        super().__init__(ErrorCode.SERVO_COMMAND_FAILED, message, details)


class ConnectionTimeoutException(WebSocketException):
    """连接超时异常"""

    def __init__(self, message: str = "", details: Optional[Dict[str, Any]] = None):
        super().__init__(ErrorCode.CONNECTION_TIMEOUT, message, details)
