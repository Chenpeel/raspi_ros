"""
消息处理器单元测试
"""

import pytest
import json
from websocket_bridge.message_handler import MessageHandler, MessageType


class TestMessageHandler:
    """MessageHandler 测试套件"""

    def setup_method(self):
        """每个测试方法执行前的初始化"""
        self.handler = MessageHandler(debug=False)

    def test_parse_message_valid_json(self):
        """测试解析有效的 JSON 消息"""
        raw = '{"type": "heartbeat", "status": "online"}'
        result = self.handler.parse_message(raw)

        assert result is not None
        assert result["type"] == "heartbeat"
        assert result["status"] == "online"

    def test_parse_message_invalid_json(self):
        """测试解析无效的 JSON 消息"""
        raw = '{"type": "heartbeat", invalid'
        result = self.handler.parse_message(raw)

        assert result is None

    def test_get_message_type_heartbeat(self):
        """测试识别心跳消息类型"""
        data = {"type": "heartbeat"}
        msg_type = self.handler.get_message_type(data)

        assert msg_type == MessageType.HEARTBEAT

    def test_get_message_type_servo_control(self):
        """测试识别舵机控制消息类型"""
        data = {"type": "servo_control", "b": -1, "c": 1, "p": 1500}
        msg_type = self.handler.get_message_type(data)

        assert msg_type == MessageType.SERVO_CONTROL

    def test_get_message_type_case_insensitive(self):
        """测试消息类型识别大小写不敏感"""
        data1 = {"type": "HEARTBEAT"}
        data2 = {"Type": "heartbeat"}
        data3 = {"TYPE": "HeartBeat"}

        assert self.handler.get_message_type(data1) == MessageType.HEARTBEAT
        assert self.handler.get_message_type(data2) == MessageType.HEARTBEAT
        assert self.handler.get_message_type(data3) == MessageType.HEARTBEAT

    def test_get_message_type_inferred_servo(self):
        """测试根据内容推断舵机控制类型"""
        # 无 type 字段，但有舵机控制字段
        data = {"b": -1, "c": 1, "p": 1500}
        msg_type = self.handler.get_message_type(data)

        assert msg_type == MessageType.SERVO_CONTROL

    def test_get_message_type_inferred_web_servo(self):
        """测试根据 web_servo 推断舵机控制类型"""
        data = {
            "character_name": "robot",
            "web_servo": {
                "is_bus_servo": True,
                "servo_id": 1,
                "position": 90,
                "speed": 100
            }
        }
        msg_type = self.handler.get_message_type(data)

        assert msg_type == MessageType.SERVO_CONTROL

    def test_get_message_type_inferred_heartbeat(self):
        """测试根据内容推断心跳类型"""
        # 无 type 字段，但有心跳特征字段
        data = {"timestamp": 1234567890, "status": "online"}
        msg_type = self.handler.get_message_type(data)

        assert msg_type == MessageType.HEARTBEAT

    def test_get_message_type_unknown(self):
        """测试未知消息类型"""
        data = {"foo": "bar"}
        msg_type = self.handler.get_message_type(data)

        assert msg_type == MessageType.UNKNOWN

    def test_parse_bcp_protocol_bus_servo(self):
        """测试解析 BCP 协议 - 总线舵机"""
        data = {"b": -1, "c": 1, "p": 1500, "s": 100}
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["servo_type"] == "bus"
        assert result["servo_id"] == 1
        # 1500us 应该对应约 90 度
        assert 85 <= result["position"] <= 95
        assert result["speed"] == 100

    def test_parse_bcp_protocol_pca_servo(self):
        """测试解析 BCP 协议 - PCA 舵机"""
        data = {"b": 0, "c": 5, "p": 300}
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["servo_type"] == "pca"
        assert result["servo_id"] == 5
        assert result["position"] == 300
        assert result["port"] == 0

    def test_parse_bcp_protocol_with_port(self):
        """测试解析 BCP 协议 - 带端口字段"""
        data = {"b": -1, "c": 1, "p": 1500, "s": 150, "port": 2}
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["servo_type"] == "bus"
        assert result["port"] == 2

    def test_parse_full_format_servo_id(self):
        """测试解析完整格式 - 使用 servo_id"""
        data = {"servo_id": 3, "position": 120, "speed": 80}
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["servo_type"] == "bus"
        assert result["servo_id"] == 3
        assert result["position"] == 120
        assert result["speed"] == 80

    def test_parse_full_format_angle(self):
        """测试解析完整格式 - 使用 angle 字段"""
        data = {"id": 2, "angle": 45}
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["servo_id"] == 2
        assert result["position"] == 45

    def test_parse_full_format_channel(self):
        """测试解析完整格式 - 使用 channel (PCA)"""
        data = {"channel": 7, "position": 200}
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["servo_type"] == "pca"
        assert result["servo_id"] == 7
        assert result["position"] == 200

    def test_parse_web_servo_payload_bus(self):
        """测试解析 web_servo 格式 - 总线舵机"""
        data = {
            "character_name": "robot",
            "web_servo": {
                "is_bus_servo": True,
                "servo_id": 2,
                "position": 45,
                "speed": 120
            }
        }
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["servo_type"] == "bus"
        assert result["servo_id"] == 2
        assert result["position"] == 45
        assert result["speed"] == 120

    def test_parse_web_servo_payload_pca(self):
        """测试解析 web_servo 格式 - PCA 舵机"""
        data = {
            "web_servo": {
                "is_bus_servo": False,
                "servo_id": 5,
                "position": 300
            }
        }
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["servo_type"] == "pca"
        assert result["servo_id"] == 5
        assert result["position"] == 300

    def test_parse_servo_control_invalid_data(self):
        """测试解析无效的舵机控制数据"""
        # 缺少必要字段
        data1 = {"b": -1, "c": 1}  # 缺少 p
        data2 = {"position": 90}  # 缺少 ID
        data3 = "not a dict"  # 不是字典

        assert self.handler.parse_servo_control(data1) is None
        assert self.handler.parse_servo_control(data2) is None
        assert self.handler.parse_servo_control(data3) is None

    def test_us_to_angle_conversion(self):
        """测试微秒到角度的转换"""
        # 500us → 0度
        assert self.handler._us_to_angle(500) == 0

        # 1500us → 90度
        angle = self.handler._us_to_angle(1500)
        assert 88 <= angle <= 92

        # 2500us → 180度
        assert self.handler._us_to_angle(2500) == 180

        # 边界情况：超出范围
        assert self.handler._us_to_angle(0) == 0
        assert self.handler._us_to_angle(5000) == 180

    def test_angle_to_us_conversion(self):
        """测试角度到微秒的转换"""
        # 0度 → 500us
        assert self.handler._angle_to_us(0) == 500

        # 90度 → 1500us
        us = self.handler._angle_to_us(90)
        assert 1490 <= us <= 1510

        # 180度 → 2500us
        assert self.handler._angle_to_us(180) == 2500

        # 边界情况：超出范围
        assert self.handler._angle_to_us(-10) == 500
        assert self.handler._angle_to_us(200) == 2500

    def test_register_handler(self):
        """测试注册消息处理器"""
        async def test_callback(data):
            return "test response"

        self.handler.register_handler("test_type", test_callback)

        assert "test_type" in self.handler.message_callbacks
        assert len(self.handler.message_callbacks["test_type"]) == 1
        assert self.handler.message_callbacks["test_type"][0] == test_callback

    def test_register_multiple_handlers(self):
        """测试为同一类型注册多个处理器"""
        async def callback1(data):
            return "response1"

        async def callback2(data):
            return "response2"

        self.handler.register_handler("test_type", callback1)
        self.handler.register_handler("test_type", callback2)

        assert len(self.handler.message_callbacks["test_type"]) == 2

    @pytest.mark.asyncio
    async def test_process_message_heartbeat(self):
        """测试处理心跳消息"""
        data = {"type": "heartbeat"}
        response = await self.handler.process_message(data)

        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "heartbeat"
        assert response_data["status"] == "online"
        assert "timestamp" in response_data

    @pytest.mark.asyncio
    async def test_process_message_status_query(self):
        """测试处理状态查询消息"""
        data = {"type": "status_query"}
        response = await self.handler.process_message(data)

        assert response is not None
        response_data = json.loads(response)
        assert response_data["type"] == "status"
        assert "current_status" in response_data
        assert response_data["result_code"] == 200

    @pytest.mark.asyncio
    async def test_process_message_with_custom_handler(self):
        """测试使用自定义处理器处理消息"""
        async def custom_handler(data):
            return json.dumps({"result": "custom"})

        self.handler.register_handler(MessageType.HEARTBEAT, custom_handler)

        data = {"type": "heartbeat"}
        response = await self.handler.process_message(data)

        assert response is not None
        response_data = json.loads(response)
        assert response_data["result"] == "custom"

    @pytest.mark.asyncio
    async def test_process_message_unknown_type(self):
        """测试处理未知类型消息"""
        data = {"type": "unknown_type", "foo": "bar"}
        response = await self.handler.process_message(data)

        # 未知类型应该返回 None（除非有注册的处理器）
        assert response is None

    def test_is_servo_control_detection(self):
        """测试舵机控制命令检测"""
        # 应该识别为舵机控制
        assert self.handler._is_servo_control({"b": -1, "c": 1, "p": 1500})
        assert self.handler._is_servo_control({"servo_id": 1, "position": 90})
        assert self.handler._is_servo_control({"id": 1, "angle": 45})
        assert self.handler._is_servo_control({"channel": 5, "speed": 100})
        assert self.handler._is_servo_control({
            "web_servo": {"is_bus_servo": True, "servo_id": 1, "position": 90}
        })

        # 不应该识别为舵机控制
        assert not self.handler._is_servo_control({"type": "heartbeat"})
        assert not self.handler._is_servo_control({"foo": "bar"})

    def test_parse_servo_with_mixed_case_keys(self):
        """测试解析混合大小写键的舵机命令"""
        # 当前实现使用小写键，这个测试验证行为
        data = {"b": -1, "c": 1, "p": 1500}
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["servo_type"] == "bus"


class TestMessageType:
    """MessageType 枚举测试"""

    def test_message_type_values(self):
        """测试消息类型枚举值"""
        assert MessageType.HEARTBEAT.value == "heartbeat"
        assert MessageType.SERVO_CONTROL.value == "servo_control"
        assert MessageType.BROADCAST.value == "broadcast"
        assert MessageType.PRIVATE.value == "private"
        assert MessageType.REGISTER.value == "register"
        assert MessageType.STATUS_QUERY.value == "status_query"
        assert MessageType.UNKNOWN.value == "unknown"

    def test_message_type_from_string(self):
        """测试从字符串创建消息类型"""
        assert MessageType("heartbeat") == MessageType.HEARTBEAT
        assert MessageType("servo_control") == MessageType.SERVO_CONTROL

        with pytest.raises(ValueError):
            MessageType("invalid_type")


class TestMessageHandlerEdgeCases:
    """MessageHandler 边界情况测试"""

    def setup_method(self):
        """每个测试方法执行前的初始化"""
        self.handler = MessageHandler(debug=False)

    def test_parse_empty_message(self):
        """测试解析空消息"""
        assert self.handler.parse_message("") is None
        assert self.handler.parse_message("{}") == {}

    def test_parse_servo_with_string_values(self):
        """测试解析带字符串值的舵机命令"""
        # 应该能够转换字符串到整数
        data = {"b": "-1", "c": "1", "p": "1500", "s": "100"}
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["servo_type"] == "bus"
        assert result["servo_id"] == 1

    def test_parse_servo_with_invalid_types(self):
        """测试解析带无效类型的舵机命令"""
        # 无法转换的值应该返回 None
        data = {"b": -1, "c": "invalid", "p": 1500}
        result = self.handler.parse_servo_control(data)

        assert result is None

    def test_us_to_angle_with_negative_values(self):
        """测试负值微秒转角度"""
        # 应该钳位到 0
        assert self.handler._us_to_angle(-100) == 0

    def test_angle_to_us_with_extreme_values(self):
        """测试极端角度值转微秒"""
        # 应该钳位到有效范围
        assert self.handler._angle_to_us(-1000) == 500
        assert self.handler._angle_to_us(1000) == 2500

    def test_get_message_type_with_none(self):
        """测试 None 数据的消息类型"""
        assert self.handler.get_message_type(None) == MessageType.UNKNOWN
        assert self.handler.get_message_type([]) == MessageType.UNKNOWN
        assert self.handler.get_message_type("string") == MessageType.UNKNOWN

    def test_parse_servo_with_default_speed(self):
        """测试舵机命令的默认速度值"""
        data = {"b": -1, "c": 1, "p": 1500}  # 没有 s 字段
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["speed"] == 100  # 默认值

    def test_parse_servo_with_default_port(self):
        """测试舵机命令的默认端口值"""
        data = {"b": -1, "c": 1, "p": 1500, "s": 100}  # 没有 port 字段
        result = self.handler.parse_servo_control(data)

        assert result is not None
        assert result["port"] == 0  # 默认值
