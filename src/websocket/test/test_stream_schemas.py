"""
流格式管理器单元测试
"""

import pytest
import json
import tempfile
import os
from pathlib import Path
from websocket_bridge.stream_schemas import StreamSchemas, get_stream_schemas


class TestStreamSchemas:
    """StreamSchemas 测试套件"""

    def setup_method(self):
        """每个测试方法执行前的初始化"""
        # 使用包内默认配置
        self.schemas = StreamSchemas()

    def test_load_web2ros_schema(self):
        """测试加载 Web → ROS 格式定义"""
        schema = self.schemas.get_web2jiyuan_schema()

        assert schema is not None
        assert isinstance(schema, dict)
        # 应该包含舵机控制相关字段
        assert "web_servo" in schema or "controller_type" in schema

    def test_load_ros2web_schema(self):
        """测试加载 ROS → Web 格式定义"""
        schema = self.schemas.get_jiyuan2web_schema()

        assert schema is not None
        assert isinstance(schema, dict)
        # 应该包含机器人状态相关字段
        assert "character_name" in schema or "servo_states" in schema

    def test_validate_web_command_bcp_protocol(self):
        """测试验证 BCP 协议命令"""
        # 简写协议 {b, c, p}
        data = {"b": -1, "c": 1, "p": 1500}
        assert self.schemas.validate_web_command(data) is True

    def test_validate_web_command_full_format(self):
        """测试验证完整格式命令"""
        data1 = {"servo_id": 1, "position": 90}
        data2 = {"id": 2, "angle": 45}
        data3 = {"channel": 5, "position": 200}

        assert self.schemas.validate_web_command(data1) is True
        assert self.schemas.validate_web_command(data2) is True
        assert self.schemas.validate_web_command(data3) is True

    def test_validate_web_command_invalid(self):
        """测试验证无效命令"""
        # 不包含舵机控制相关字段
        data1 = {"foo": "bar"}
        data2 = {"type": "heartbeat"}
        data3 = "not a dict"

        assert self.schemas.validate_web_command(data1) is False
        assert self.schemas.validate_web_command(data2) is False
        assert self.schemas.validate_web_command(data3) is False

    def test_validate_jiyuan_response_valid(self):
        """测试验证有效的机器人响应"""
        data = {
            "character_name": "robot",
            "current_status": {
                "movement_active": False,
                "action": "idle"
            },
            "result_code": 200
        }

        assert self.schemas.validate_jiyuan_response(data) is True

    def test_validate_jiyuan_response_missing_fields(self):
        """测试验证缺少字段的机器人响应"""
        # 缺少 character_name
        data1 = {"current_status": {"action": "idle"}}
        # 缺少 current_status
        data2 = {"character_name": "robot"}
        # 不是字典
        data3 = "not a dict"

        assert self.schemas.validate_jiyuan_response(data1) is False
        assert self.schemas.validate_jiyuan_response(data2) is False
        assert self.schemas.validate_jiyuan_response(data3) is False

    def test_create_servo_command_bus_servo(self):
        """测试创建总线舵机命令"""
        cmd = self.schemas.create_servo_command(
            servo_type="bus",
            servo_id=1,
            position=90,
            speed=150,
            port=2
        )

        assert cmd["b"] == -1
        assert cmd["c"] == 1
        assert cmd["p"] == 90
        assert cmd["s"] == 150
        assert cmd["port"] == 2

    def test_create_servo_command_pca_servo(self):
        """测试创建 PCA 舵机命令"""
        cmd = self.schemas.create_servo_command(
            servo_type="pca",
            servo_id=5,
            position=300
        )

        assert cmd["b"] == 0
        assert cmd["c"] == 5
        assert cmd["p"] == 300

    def test_create_servo_command_default_values(self):
        """测试创建舵机命令的默认值"""
        cmd = self.schemas.create_servo_command(
            servo_type="bus",
            servo_id=1,
            position=90
        )

        # 应该有默认的 speed 和 port
        assert cmd["s"] == 100  # 默认速度
        assert cmd["port"] == 0  # 默认端口

    def test_create_jiyuan_response_basic(self):
        """测试创建基本的机器人响应"""
        response = self.schemas.create_jiyuan_response(
            status_code=200,
            message="success"
        )

        assert response["character_name"] == "robot"
        assert response["result_code"] == 200
        assert "current_status" in response
        assert "time" in response

    def test_create_jiyuan_response_with_servo_states(self):
        """测试创建包含舵机状态的响应"""
        servo_states = {
            "bus_servos": {"id_1": 90, "id_2": 120},
            "pwm_servos": {"id_3": 300}
        }

        response = self.schemas.create_jiyuan_response(
            status_code=200,
            servo_states=servo_states
        )

        assert response["bus_servos"] == servo_states["bus_servos"]
        assert response["pwm_servos"] == servo_states["pwm_servos"]

    def test_create_jiyuan_response_with_system_info(self):
        """测试创建包含系统信息的响应"""
        system_info = {
            "cpu_temp": 45.5,
            "free_memory": 2048
        }

        response = self.schemas.create_jiyuan_response(
            status_code=200,
            system_info=system_info
        )

        assert response["cpu_temp"] == 45.5
        assert response["free_memory"] == 2048

    def test_create_jiyuan_response_default_servo_states(self):
        """测试响应的默认舵机状态"""
        response = self.schemas.create_jiyuan_response(status_code=200)

        assert response["bus_servos"] == {}
        assert response["pwm_servos"] == {}

    def test_create_jiyuan_response_default_current_status(self):
        """测试响应的默认当前状态"""
        response = self.schemas.create_jiyuan_response(status_code=200)

        assert "current_status" in response
        assert response["current_status"]["movement_active"] is False
        assert response["current_status"]["listening"] is False
        assert response["current_status"]["action"] == "idle"
        assert response["current_status"]["led_state"] == "off"


class TestStreamSchemasWithCustomConfig:
    """使用自定义配置的 StreamSchemas 测试"""

    def test_load_from_custom_config_dir(self):
        """测试从自定义配置目录加载"""
        # 创建临时配置目录
        with tempfile.TemporaryDirectory() as tmpdir:
            # 创建测试配置文件
            web2ros_config = {
                "character_name": "test_robot",
                "controller_type": "web_servo"
            }
            ros2web_config = {
                "character_name": "test_robot",
                "servo_states": {}
            }

            # 写入配置文件
            with open(os.path.join(tmpdir, "std_web2ros_stream.json"), 'w') as f:
                json.dump(web2ros_config, f)
            with open(os.path.join(tmpdir, "std_ros2web_stream.json"), 'w') as f:
                json.dump(ros2web_config, f)

            # 使用自定义配置目录初始化
            schemas = StreamSchemas(config_dir=tmpdir)

            # 验证加载的配置
            assert schemas.get_web2jiyuan_schema()["character_name"] == "test_robot"
            assert schemas.get_jiyuan2web_schema()["character_name"] == "test_robot"

    def test_load_from_nonexistent_dir(self):
        """测试从不存在的目录加载"""
        # 应该返回空字典而不抛出异常
        schemas = StreamSchemas(config_dir="/nonexistent/dir")

        assert schemas.get_web2jiyuan_schema() == {}
        assert schemas.get_jiyuan2web_schema() == {}

    def test_load_invalid_json_file(self):
        """测试加载无效的 JSON 文件"""
        with tempfile.TemporaryDirectory() as tmpdir:
            # 创建无效的 JSON 文件
            invalid_file = os.path.join(tmpdir, "std_web2ros_stream.json")
            with open(invalid_file, 'w') as f:
                f.write("{invalid json")

            # 创建有效的文件以避免两个都失败
            valid_file = os.path.join(tmpdir, "std_ros2web_stream.json")
            with open(valid_file, 'w') as f:
                json.dump({"test": "data"}, f)

            schemas = StreamSchemas(config_dir=tmpdir)

            # 应该返回空字典而不抛出异常
            assert schemas.get_web2jiyuan_schema() == {}
            assert schemas.get_jiyuan2web_schema() == {"test": "data"}


class TestGetStreamSchemas:
    """全局实例函数测试"""

    def test_get_stream_schemas_singleton(self):
        """测试全局实例是单例模式"""
        instance1 = get_stream_schemas()
        instance2 = get_stream_schemas()

        # 应该返回同一个实例
        assert instance1 is instance2

    def test_get_stream_schemas_returns_valid_instance(self):
        """测试全局实例返回有效的 StreamSchemas 对象"""
        instance = get_stream_schemas()

        assert isinstance(instance, StreamSchemas)
        assert hasattr(instance, 'get_web2jiyuan_schema')
        assert hasattr(instance, 'get_jiyuan2web_schema')


class TestStreamSchemasEdgeCases:
    """StreamSchemas 边界情况测试"""

    def setup_method(self):
        """每个测试方法执行前的初始化"""
        self.schemas = StreamSchemas()

    def test_validate_empty_dict(self):
        """测试验证空字典"""
        assert self.schemas.validate_web_command({}) is False
        assert self.schemas.validate_jiyuan_response({}) is False

    def test_create_servo_command_with_zero_values(self):
        """测试创建零值舵机命令"""
        cmd = self.schemas.create_servo_command(
            servo_type="bus",
            servo_id=0,
            position=0,
            speed=0,
            port=0
        )

        assert cmd["c"] == 0
        assert cmd["p"] == 0
        assert cmd["s"] == 0
        assert cmd["port"] == 0

    def test_create_servo_command_with_extreme_values(self):
        """测试创建极端值舵机命令"""
        cmd = self.schemas.create_servo_command(
            servo_type="bus",
            servo_id=254,
            position=180,
            speed=255
        )

        assert cmd["c"] == 254
        assert cmd["p"] == 180
        assert cmd["s"] == 255

    def test_validate_web_command_with_partial_fields(self):
        """测试验证部分字段的命令"""
        # 只有 b 和 c，缺少 p
        data1 = {"b": -1, "c": 1}
        # 只有 id，缺少 position
        data2 = {"id": 1}

        # 只要包含任何舵机相关字段就通过验证
        assert self.schemas.validate_web_command(data1) is True
        assert self.schemas.validate_web_command(data2) is True

    def test_create_jiyuan_response_with_error_code(self):
        """测试创建错误状态码的响应"""
        response = self.schemas.create_jiyuan_response(
            status_code=500,
            message="internal error"
        )

        assert response["result_code"] == 500

    def test_create_jiyuan_response_empty_servo_states(self):
        """测试创建空舵机状态的响应"""
        servo_states = {
            "bus_servos": {},
            "pwm_servos": {}
        }

        response = self.schemas.create_jiyuan_response(
            status_code=200,
            servo_states=servo_states
        )

        assert response["bus_servos"] == {}
        assert response["pwm_servos"] == {}

    def test_create_servo_command_pca_ignores_speed(self):
        """测试 PCA 舵机命令忽略速度参数"""
        cmd = self.schemas.create_servo_command(
            servo_type="pca",
            servo_id=5,
            position=300,
            speed=150  # PCA 舵机不使用速度
        )

        # 命令中仍然包含 s 字段（但 PCA 舵机会忽略它）
        assert cmd["b"] == 0
        assert "s" in cmd
