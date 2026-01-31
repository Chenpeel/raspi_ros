"""
WebSocket 处理器单元测试
"""

import pytest
import json
import time
from websocket_bridge.websocket_handler import WebSocketHandler
from websocket_bridge.message_handler import MessageType


class TestWebSocketHandler:
    """WebSocketHandler 测试套件"""

    def setup_method(self):
        """每个测试方法执行前的初始化"""
        self.handler = WebSocketHandler(device_id="test_device", debug=False)

    @pytest.mark.asyncio
    async def test_handle_heartbeat_message(self):
        """测试处理心跳消息"""
        raw_message = json.dumps({"type": "heartbeat"})
        response = await self.handler.handle_message(raw_message)

        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "heartbeat"
        assert response_data["status"] == "online"
        assert response_data["device_id"] == "test_device"
        assert "timestamp" in response_data

    @pytest.mark.asyncio
    async def test_handle_heartbeat_updates_timestamp(self):
        """测试心跳消息更新时间戳"""
        initial_time = self.handler.last_heartbeat
        time.sleep(0.1)

        raw_message = json.dumps({"type": "heartbeat"})
        await self.handler.handle_message(raw_message)

        assert self.handler.last_heartbeat > initial_time

    @pytest.mark.asyncio
    async def test_handle_servo_control_message(self):
        """测试处理舵机控制消息"""
        raw_message = json.dumps({
            "type": "servo_control",
            "b": -1,
            "c": 1,
            "p": 1500,
            "s": 100
        })
        response = await self.handler.handle_message(raw_message)

        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "servo_control_ack"
        assert response_data["status"] == "accepted"
        assert "command" in response_data
        assert response_data["command"]["servo_type"] == "bus"

    @pytest.mark.asyncio
    async def test_handle_web_servo_message(self):
        """测试处理 web_servo 格式的舵机控制消息"""
        raw_message = json.dumps({
            "character_name": "robot",
            "web_servo": {
                "is_bus_servo": True,
                "servo_id": 3,
                "position": 120,
                "speed": 90
            },
            "action": {}
        })
        response = await self.handler.handle_message(raw_message)

        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "servo_control_ack"
        assert response_data["status"] == "accepted"
        assert response_data["command"]["servo_id"] == 3
        assert response_data["command"]["servo_type"] == "bus"

    @pytest.mark.asyncio
    async def test_handle_invalid_servo_control(self):
        """测试处理无效的舵机控制消息"""
        raw_message = json.dumps({
            "type": "servo_control",
            "invalid": "data"
        })
        response = await self.handler.handle_message(raw_message)

        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "error"
        assert response_data["error_code"] == "invalid_servo_command"

    @pytest.mark.asyncio
    async def test_handle_servo_control_with_callback(self):
        """测试舵机控制消息调用回调函数"""
        callback_called = False
        received_command = None

        async def servo_callback(servo_cmd):
            nonlocal callback_called, received_command
            callback_called = True
            received_command = servo_cmd

        self.handler.register_servo_command_handler(servo_callback)

        raw_message = json.dumps({
            "type": "servo_control",
            "b": -1,
            "c": 5,
            "p": 2000,
            "s": 150
        })
        await self.handler.handle_message(raw_message)

        assert callback_called
        assert received_command is not None
        assert received_command["servo_id"] == 5

    @pytest.mark.asyncio
    async def test_handle_status_query_message(self):
        """测试处理状态查询消息"""
        raw_message = json.dumps({"type": "status_query"})
        response = await self.handler.handle_message(raw_message)

        assert response is not None
        response_data = json.loads(response)
        assert "character_name" in response_data
        assert "current_status" in response_data
        assert response_data["result_code"] == 200

    @pytest.mark.asyncio
    async def test_handle_status_query_with_callback(self):
        """测试状态查询消息调用回调函数"""
        async def status_callback():
            return {
                "character_name": "custom_robot",
                "current_status": {"action": "walking"},
                "result_code": 200
            }

        self.handler.register_status_query_handler(status_callback)

        raw_message = json.dumps({"type": "status_query"})
        response = await self.handler.handle_message(raw_message)

        response_data = json.loads(response)
        assert response_data["character_name"] == "custom_robot"
        assert response_data["current_status"]["action"] == "walking"

    @pytest.mark.asyncio
    async def test_handle_register_message(self):
        """测试处理注册消息"""
        raw_message = json.dumps({
            "type": "register",
            "name": "my_device"
        })
        response = await self.handler.handle_message(raw_message)

        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "register_ack"
        assert response_data["status"] == "success"
        assert response_data["device_name"] == "my_device"
        assert "supported_commands" in response_data

    @pytest.mark.asyncio
    async def test_handle_broadcast_message(self):
        """测试处理广播消息"""
        callback_called = False

        async def servo_callback(servo_cmd):
            nonlocal callback_called
            callback_called = True

        self.handler.register_servo_command_handler(servo_callback)

        raw_message = json.dumps({
            "type": "broadcast",
            "content": {
                "b": -1,
                "c": 3,
                "p": 1800
            }
        })
        response = await self.handler.handle_message(raw_message)

        # 广播消息不返回响应
        assert response is None
        # 但应该调用舵机控制回调
        assert callback_called

    @pytest.mark.asyncio
    async def test_handle_private_message(self):
        """测试处理私有消息"""
        callback_called = False

        async def servo_callback(servo_cmd):
            nonlocal callback_called
            callback_called = True

        self.handler.register_servo_command_handler(servo_callback)

        raw_message = json.dumps({
            "type": "private",
            "content": {
                "b": 0,
                "c": 7,
                "p": 300
            }
        })
        response = await self.handler.handle_message(raw_message)

        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "private_ack"
        assert response_data["status"] == "processed"
        assert callback_called

    @pytest.mark.asyncio
    async def test_handle_invalid_json(self):
        """测试处理无效 JSON"""
        raw_message = "{invalid json"
        response = await self.handler.handle_message(raw_message)

        # 无效 JSON 应该返回 None
        assert response is None

    @pytest.mark.asyncio
    async def test_handle_unknown_message_type(self):
        """测试处理未知消息类型"""
        raw_message = json.dumps({"type": "unknown_type"})
        response = await self.handler.handle_message(raw_message)

        # 未知类型应该返回 None
        assert response is None

    def test_update_servo_state_bus_servo(self):
        """测试更新总线舵机状态"""
        self.handler.update_servo_state(1, "bus", 90)

        assert "bus_servos" in self.handler.servo_state
        assert "id_1" in self.handler.servo_state["bus_servos"]
        assert self.handler.servo_state["bus_servos"]["id_1"] == 90

    def test_update_servo_state_pca_servo(self):
        """测试更新 PCA 舵机状态"""
        self.handler.update_servo_state(5, "pca", 300)

        assert "pwm_servos" in self.handler.servo_state
        assert "id_5" in self.handler.servo_state["pwm_servos"]
        assert self.handler.servo_state["pwm_servos"]["id_5"] == 300

    def test_update_servo_state_multiple_servos(self):
        """测试更新多个舵机状态"""
        self.handler.update_servo_state(1, "bus", 90)
        self.handler.update_servo_state(2, "bus", 120)
        self.handler.update_servo_state(3, "pca", 200)

        assert len(self.handler.servo_state["bus_servos"]) == 2
        assert len(self.handler.servo_state["pwm_servos"]) == 1

    def test_update_system_state(self):
        """测试更新系统状态"""
        self.handler.update_system_state(
            cpu_temp=45.5,
            free_memory=2048,
            uptime=3600
        )

        assert self.handler.system_state["cpu_temp"] == 45.5
        assert self.handler.system_state["free_memory"] == 2048
        assert self.handler.system_state["uptime"] == 3600

    def test_get_uptime(self):
        """测试获取运行时间"""
        # 设置已知的最后心跳时间
        self.handler.last_heartbeat = time.time() - 10

        uptime = self.handler.get_uptime()

        # 应该约为 10 秒
        assert 9.5 <= uptime <= 10.5

    def test_register_message_handler(self):
        """测试注册自定义消息处理器"""
        async def custom_handler(data):
            return json.dumps({"custom": "response"})

        self.handler.register_message_handler("custom_type", custom_handler)

        # 验证处理器已注册到底层 MessageHandler
        assert "custom_type" in self.handler.message_handler.message_callbacks

    @pytest.mark.asyncio
    async def test_heartbeat_callback_exception_handling(self):
        """测试心跳回调异常处理"""
        async def failing_callback():
            raise Exception("Test exception")

        self.handler.register_heartbeat_handler(failing_callback)

        raw_message = json.dumps({"type": "heartbeat"})
        # 不应该抛出异常
        response = await self.handler.handle_message(raw_message)

        # 应该仍然返回心跳响应
        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "heartbeat"

    @pytest.mark.asyncio
    async def test_servo_command_callback_exception_handling(self):
        """测试舵机命令回调异常处理"""
        async def failing_callback(servo_cmd):
            raise Exception("Test exception")

        self.handler.register_servo_command_handler(failing_callback)

        raw_message = json.dumps({
            "type": "servo_control",
            "b": -1,
            "c": 1,
            "p": 1500
        })
        response = await self.handler.handle_message(raw_message)

        # 应该返回错误响应
        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "error"
        assert response_data["error_code"] == "servo_command_failed"

    @pytest.mark.asyncio
    async def test_status_query_callback_exception_handling(self):
        """测试状态查询回调异常处理"""
        async def failing_callback():
            raise Exception("Test exception")

        self.handler.register_status_query_handler(failing_callback)

        raw_message = json.dumps({"type": "status_query"})
        # 不应该抛出异常
        response = await self.handler.handle_message(raw_message)

        # 应该返回默认状态响应
        assert response is not None
        response_data = json.loads(response)
        assert "character_name" in response_data


class TestWebSocketHandlerEdgeCases:
    """WebSocketHandler 边界情况测试"""

    def setup_method(self):
        """每个测试方法执行前的初始化"""
        self.handler = WebSocketHandler(device_id="edge_test", debug=False)

    @pytest.mark.asyncio
    async def test_handle_empty_message(self):
        """测试处理空消息"""
        response = await self.handler.handle_message("")
        assert response is None

    @pytest.mark.asyncio
    async def test_handle_empty_dict_message(self):
        """测试处理空字典消息"""
        raw_message = json.dumps({})
        response = await self.handler.handle_message(raw_message)

        # 空字典应该被识别为 UNKNOWN 类型
        assert response is None

    @pytest.mark.asyncio
    async def test_servo_control_without_callback(self):
        """测试未注册回调时的舵机控制"""
        raw_message = json.dumps({
            "type": "servo_control",
            "b": -1,
            "c": 1,
            "p": 1500
        })
        response = await self.handler.handle_message(raw_message)

        # 即使没有回调，也应该返回确认
        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "servo_control_ack"

    @pytest.mark.asyncio
    async def test_status_query_returns_servo_state(self):
        """测试状态查询返回舵机状态"""
        # 先更新一些舵机状态
        self.handler.update_servo_state(1, "bus", 90)
        self.handler.update_servo_state(2, "pca", 300)

        raw_message = json.dumps({"type": "status_query"})
        response = await self.handler.handle_message(raw_message)

        response_data = json.loads(response)
        assert "bus_servos" in response_data
        assert "pwm_servos" in response_data
        assert response_data["bus_servos"]["id_1"] == 90
        assert response_data["pwm_servos"]["id_2"] == 300

    def test_get_supported_commands(self):
        """测试获取支持的命令列表"""
        commands = self.handler._get_supported_commands()

        assert "servo_control" in commands
        assert "heartbeat" in commands
        assert "status_query" in commands
        assert isinstance(commands["servo_control"], list)
        assert len(commands["servo_control"]) > 0

    @pytest.mark.asyncio
    async def test_broadcast_without_servo_content(self):
        """测试不包含舵机命令的广播消息"""
        raw_message = json.dumps({
            "type": "broadcast",
            "content": {"message": "hello"}
        })
        response = await self.handler.handle_message(raw_message)

        # 不包含舵机命令的广播应该返回 None
        assert response is None

    @pytest.mark.asyncio
    async def test_private_without_servo_content(self):
        """测试不包含舵机命令的私有消息"""
        raw_message = json.dumps({
            "type": "private",
            "content": {"message": "hello"}
        })
        response = await self.handler.handle_message(raw_message)

        # 不包含舵机命令的私有消息应该返回 None
        assert response is None

    def test_initial_servo_state_empty(self):
        """测试初始舵机状态为空"""
        handler = WebSocketHandler()
        assert handler.servo_state == {}

    def test_initial_system_state_has_defaults(self):
        """测试初始系统状态有默认值"""
        handler = WebSocketHandler()
        assert "cpu_temp" in handler.system_state
        assert "free_memory" in handler.system_state
        assert "uptime" in handler.system_state
        assert handler.system_state["cpu_temp"] == 0
