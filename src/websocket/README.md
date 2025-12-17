# WebSocket Bridge

WebSocket通信桥接包，用于ROS 2与WebSocket客户端之间的双向通信。

## 功能特性

- ✅ **双向通信**: Web/树莓派 ↔ WebSocket ↔ ROS 2
- ✅ **多客户端支持**: 维护多个WebSocket连接
- ✅ **协议转换**: JSON消息 ↔ ROS 2结构化数据
- ✅ **舵机控制**: 支持总线舵机和PCA舵机
- ✅ **状态同步**: 实时广播机器人状态
- ✅ **心跳机制**: 自动检测连接状态
- ✅ **错误处理**: 统一的错误码和异常处理
- ✅ **日志系统**: 结构化日志记录

## 快速开始

### 1. 安装依赖

```bash
pip install websockets
```

### 2. 集成到ROS 2节点

```python
from websocket_bridge import WebSocketBridgeServer

class YourNode(Node):
    def __init__(self):
        super().__init__('your_node')

        # 创建WebSocket服务器
        self.ws_server = WebSocketBridgeServer(
            host='0.0.0.0',
            port=9102,
            device_id='robot_01'
        )

        # 注册舵机控制回调
        self.ws_server.set_servo_command_callback(self.handle_servo_command)

        # 启动WebSocket服务器
        asyncio.create_task(self.ws_server.start())

    async def handle_servo_command(self, servo_cmd: dict):
        """处理舵机控制命令"""
        servo_type = servo_cmd['servo_type']  # 'bus' 或 'pca'
        servo_id = servo_cmd['servo_id']
        position = servo_cmd['position']
        # 发布到ROS 2话题...
```

### 3. 独立运行（测试）

```bash
cd src/websocket
python -m websocket_bridge.ws_server --host 0.0.0.0 --port 9102 --debug
```

## 消息格式

### Web → ROS（舵机控制）

简写格式:
```json
{
  "b": -1,      // -1=总线舵机, 0=PCA舵机
  "c": 1,       // 舵机ID
  "p": 1500,    // 位置（总线舵机为微秒值，PCA为tick值）
  "s": 100      // 速度（可选）
}
```

完整格式:
```json
{
  "servo_type": "bus",
  "servo_id": 1,
  "position": 90,
  "speed": 100
}
```

### ROS → Web（状态响应）

```json
{
  "character_name": "robot",
  "servo_states": {
    "bus_servos": [{"servo_id": 1, "angle": 90, "status": "ok"}],
    "pwm_servos": [{"servo_id": 0, "angle": 120, "status": "ok"}]
  },
  "system_state": {
    "movement_active": false,
    "action": "idle"
  },
  "timestamp": 1234567890,
  "result_code": 200
}
```

## API文档

### WebSocketBridgeServer

主要方法:

- `set_servo_command_callback(callback)` - 注册舵机命令回调
- `set_heartbeat_callback(callback)` - 注册心跳回调
- `broadcast_status(status_dict)` - 广播状态到所有客户端
- `start()` - 启动WebSocket服务器
- `stop()` - 停止服务器

### WebSocketHandler

消息处理器，负责解析和路由WebSocket消息。

### MessageHandler

消息解析器，支持多种舵机控制协议格式。

### ErrorCode

错误码枚举:

- `200` - 成功
- `4001` - 无效消息格式
- `4002` - 无效舵机命令
- `5001` - 舵机命令执行失败
- `6001` - 连接超时

## 配置

配置文件位于 `config/` 目录:

- `std_web2ros_stream.json` - Web→ROS数据流格式定义
- `std_ros2web_stream.json` - ROS→Web数据流格式定义

## 测试

运行单元测试:

```bash
cd src/websocket
pytest test/ -v
```

运行特定测试:

```bash
pytest test/test_message_handler.py -v
pytest test/test_websocket_handler.py -v
pytest test/test_stream_schemas.py -v
```

## 日志配置

```python
from websocket_bridge.logger import set_log_level
import logging

# 设置日志级别为DEBUG
set_log_level('websocket_bridge.message_handler', logging.DEBUG)
```

## 项目结构

```
websocket_bridge/
├── __init__.py                  # 包导出
├── message_handler.py           # 消息解析和路由
├── websocket_handler.py         # WebSocket处理器
├── ws_server.py                 # WebSocket服务器
├── stream_schemas.py            # JSON格式验证
├── error_codes.py               # 错误码定义
└── logger.py                    # 日志配置
```

## 许可证

MIT License

## 维护者

chenpeel (chenpeel@foxmail.com)
