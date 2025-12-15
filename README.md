# ROS 2 舵机 WebSocket 集成系统

## 架构概述

```
树莓派 (Raspberry Pi 4/5)
    ↓ (WebSocket 连接)
[WebSocket 服务器 9102 端口]
    ↓ (ROS 2 消息)
[servo_control 节点]
    ↓ ↑
[servo_core 节点]
[servo_hardware 驱动]
```

## 功能模块

### 1. **websocket_bridge** - WebSocket 通信层
- **ws_server.py**: WebSocket 服务器，监听 9102 端口
- **websocket_handler.py**: 消息路由和处理
- **message_handler.py**: JSON 消息解析和验证
- **stream_schemas.py**: 标准 JSON 流格式定义

**配置文件**:
- `std_web2ros_stream.json` - Web → ROS 消息格式
- `std_ros2web_stream.json` - ROS → Web 响应格式

### 2. **servo_control** - 集成控制节点
- **servo_control_node.py**: 集成异步 WebSocket 和同步 ROS 2
  - 启动 WebSocket 服务器
  - 转发舵机命令到 ROS 2 话题
  - 汇聚系统状态反馈

### 3. **servo_core** - 核心功能
- **core.py**: ROS 2 节点和状态管理
  - `ServoCoreNode`: ROS 2 节点基类
  - `StatusManager`: 系统状态管理

### 4. **servo_hardware** - 硬件驱动接口
- 预留接口用于直接控制舵机
- 与树莓派上的 `pi/server.py` 逻辑对应

---

## 消息格式

### Web → ROS (std_web2ros_stream.json)

**简写协议** (兼容 duck):
```json
{
  "b": -1 | 0,        // -1: 总线舵机, 0: PCA 舵机
  "c": 1,             // 舵机 ID 或通道
  "p": 1500,          // 位置 (微秒/tick)
  "s": 100            // 速度 (可选)
}
```

**完整格式**:
```json
{
  "type": "servo_control",
  "servo_type": "bus | pca",
  "servo_id": 1,
  "position": 90,
  "speed": 100
}
```

### ROS → Web (std_ros2web_stream.json)

```json
{
  "character_name": "robot",
  "bus_servos": { "id_1": 90, "id_2": 90 },
  "pwm_servos": { "id_0": 90 },
  "time": "2025-11-28-12:00:00",
  "current_status": {
    "movement_active": false,
    "listening": false,
    "action": "idle",
    "led_state": "off"
  },
  "result_code": 200
}
```

---

## 安装和编译

### 前置条件
- ROS 2 Jazzy
- Python 3.10+
- 树莓派上运行 `pi/server.py`

### 编译步骤

```bash
# 1. 进入工作区
cd ~/work/repo/jiyuan/ros

# 2. 安装依赖
rosdep install -i --from-paths src --rosdistro jazzy -y

# 3. 编译包
colcon build --packages-select websocket_bridge servo_core servo_control servo_hardware

# 4. 源配置
source install/setup.bash
```

---

## 运行方式

### 方式一：使用 Launch 文件

```bash
# 启动整个系统
ros2 launch servo_core servo_system.launch.py

# 带参数启动
ros2 launch servo_core servo_system.launch.py \
  ws_host:=0.0.0.0 \
  ws_port:=9102 \
  device_id:=default \
  debug:=true
```

### 方式二：手动启动节点

```bash
# 启动舵机控制节点 (包含 WebSocket 服务器)
ros2 run servo_control servo_control_node
```

### 方式三：独立运行 WebSocket 服务器

```bash
# 仅启动 WebSocket 服务器（用于调试）
python3 -m websocket_bridge.ws_server --host 0.0.0.0 --port 9102 --debug
```

---

## ROS 2 话题

### 发布话题

| 话题                 | 类型     | 说明                |
| -------------------- | -------- | ------------------- |
| `/servo/command`     | `String` | 舵机控制命令 (JSON) |
| `/system/state`      | `String` | 系统状态信息 (JSON) |

### 订阅话题

| 话题             | 类型     | 说明                |
| ---------------- | -------- | ------------------- |
| `/servo/state`   | `String` | 舵机状态反馈 (JSON) |

---

## 树莓派配置

### 树莓派上的 WebSocket 客户端配置

修改 `/home/chenpeel/work/repo/new-human-pi/pi/server.py`:

```python
ws_url = "ws://主机IP:9102/ws"  # 改为主机的 IP 和端口
```

### 树莓派消息流向

```
树莓派 pi/server.py (客户端)
    ↓ (连接到主机)
主机 servo_control_node
    ↓ (转发舵机命令)
舵机驱动硬件
    ↓ (状态反馈)
主机 servo_control_node
    ↓ (广播回给树莓派)
树莓派 pi/server.py
```

---

## 调试方法

### 启用调试模式

```bash
ros2 launch servo_core servo_system.launch.py debug:=true
```

### 查看 WebSocket 连接

```bash
# 列出所有活跃 ROS 2 节点
ros2 node list

# 查看 WebSocket 服务状态
ros2 topic echo /servo/command
```

### 测试 WebSocket 连接

```bash
# 使用 websocat (需先安装)
websocat ws://localhost:9102/ws

# 发送舵机命令
{"b": -1, "c": 1, "p": 1500}
```

---

## 文件结构

```
ros/
├── src/
│   ├── core/
│   │   ├── servo_core/
│   │   │   ├── __init__.py
│   │   │   └── core.py                 # 核心 ROS 2 节点
│   │   ├── config/
│   │   │   └── servo_params.yaml       # 参数配置
│   │   ├── launch/
│   │   │   └── servo_system.launch.py  # Launch 文件
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── control/
│   │   ├── servo_control/
│   │   │   ├── __init__.py
│   │   │   └── servo_control_node.py   # 集成控制节点
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── hardware/
│   │   ├── servo_hardware/
│   │   │   └── __init__.py
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── websocket/
│       ├── websocket_bridge/
│       │   ├── __init__.py
│       │   ├── ws_server.py            # WebSocket 服务器
│       │   ├── websocket_handler.py    # 消息处理
│       │   ├── message_handler.py      # 消息解析
│       │   └── stream_schemas.py       # JSON 格式
│       ├── config/
│       │   ├── std_web2ros_stream.json
│       │   └── std_ros2web_stream.json
│       ├── package.xml
│       └── setup.py
│
├── build/
├── install/
├── log/
└── README.md
```

---

## 常见问题

### Q1: WebSocket 连接失败
**解决**: 检查防火墙是否开放 9102 端口
```bash
sudo ufw allow 9102
```

### Q2: 舵机命令未生效
**解决**: 检查 ROS 2 话题是否正常通信
```bash
ros2 topic echo /servo/command
```

### Q3: 树莓派连接超时
**解决**: 确认主机 IP 和树莓派网络连通
```bash
ping 主机IP
```

---

## 贡献和扩展

- 可在 `servo_hardware` 中添加具体的舵机驱动实现
- 可在 `servo_control` 中添加更复杂的运动规划逻辑
- 可扩展 WebSocket 消息类型以支持更多功能

---

## 许可证

BSD-2.0

## 维护者

chenpeel <chgenpeel@foxmail.com>
