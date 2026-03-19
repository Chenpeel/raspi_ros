# ROS 2 舵机控制系统

基于ROS 2 Jazzy的WebSocket舵机控制系统，支持多串口并发控制、总线舵机和PCA9685舵机。

## 系统特性

- ✅ **WebSocket远程控制**: 通过WebSocket接口（9105端口）接收Web客户端命令
- ✅ **多串口并发**: 支持4个独立串口同时控制14个舵机
- ✅ **智能ID路由**: 基于Set的O(1)算法，零延迟增加
- ✅ **协议自动识别**: 支持众灵/幻尔总线舵机，未知ID按需探测并缓存
- ✅ **双舵机类型**: 支持总线舵机和PCA9685 PWM舵机
- ✅ **话题统一**: 通过 `bus_protocol_router` 统一入口 `/servo/command` 与出口 `/servo/state`
- ✅ **仿真集成桥接**: 支持 `/sim/servo_command` 与 `/sim/servo_state` 双向转发
- ✅ **Docker部署**: 支持开发/生产/手动调试三种模式
- ✅ **实时反馈**: 舵机状态实时反馈到Web客户端

## 多串口系统架构

```
Web客户端 → WebSocket(9105) → bridge_node → /servo/command
    ↓
bus_protocol_router（协议识别 + ID路由）
    ├─ bus_port_driver_0 → ttyAMA0
    ├─ bus_port_driver_1 → ttyAMA1
    ├─ bus_port_driver_2 → ttyAMA2
    └─ bus_port_driver_3 → ttyAMA3 
    ↓
多个个总线舵机硬件
```

**性能指标**:

- 并发性能: 4倍提升（4个舵机可同时运动）
- 总带宽: 460800 bps（4×115200）
- ID路由延迟: <0.0001ms（可完全忽略）
- 可靠性: 分布式架构，单串口故障不影响其他

---

## 消息格式

### Web → ROS2 (舵机控制)

**前端简写格式**:
```json
{
  "b": -1,        // -1: 总线舵机, 其他: PCA舵机
  "c": 78,        // 舵机ID/通道
  "p": 1308,      // 位置值
  "s": 100        // 速度(可选)
}
```

**ROS2标准格式** (前端自动转换):
```json
{
  "type": "servo_control",
  "servo_type": "bus",
  "servo_id": 78,
  "position": 1308,
  "speed": 100
}
```

### ROS2 → Web (舵机状态)

```json
{
  "type": "status_update",
  "device_id": "robot",
  "data": {
    "servo_type": "bus",
    "servo_id": 78,
    "position": 1308,
    "load": 0,
    "temperature": 25,
    "error_code": 0,
    "timestamp": 1765872354.567
  },
  "timestamp": 1765872354
}
```

### 心跳消息

```json
{
  "type": "heartbeat",
  "status": "online",
  "device_id": "robot",
  "timestamp": 1765872354
}
```



---

## 安装和编译

### 前置条件

- **操作系统**: Ubuntu 24.04 或 Raspberry Pi OS
- **ROS 2**: Jazzy
- **Python**: 3.12+
- **硬件**:
  - 树莓派4/5 (推荐5B)
  - HiWonder总线舵机 (可选)
  - PCA9685舵机驱动板 (可选)

### Docker部署 (推荐)

```bash
# 1. 构建镜像
cd /path/to/{this_repo}
docker compose build

# 2. （升级后推荐）先清理旧容器，避免端口冲突
docker compose --profile dev --profile production --profile manual down --remove-orphans

# 3. 启动开发容器
docker compose --profile dev up -d ros2_servo_dev

# 4. 进入容器
docker compose exec ros2_servo_dev bash

# 5. 编译
source /opt/ros/jazzy/setup.bash
colcon build

# 6. 运行
bash scripts/init.sh
```

### 本地编译

```bash
# 1. 安装依赖
sudo apt update
sudo apt install -y python3-pip python3-websockets

# 2. 进入工作区
cd /path/to/{this_repo}

# 3. 安装ROS依赖
rosdep install -i --from-paths src --rosdistro jazzy -y

# 4. 编译包
colcon build

# 5. 源配置
source install/setup.bash
```

---

## 运行方式

### 方式一: 使用Launch文件 (推荐)

```bash
# 启动完整系统 (默认9105端口，调试关闭)
ros2 launch websocket_bridge full_system.launch.py

# 自定义参数启动
ros2 launch websocket_bridge full_system.launch.py \
  ws_host:=0.0.0.0 \
  ws_port:=9105 \
  device_id:=robot \
  enable_isaac_bridge:=true \
  debug:=true \
  baudrate:=115200
```

### 方式二: 使用初始化脚本

```bash
# Docker容器内
bash scripts/init.sh

# 或手动执行
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch websocket_bridge full_system.launch.py
```

### 方式三: 独立启动节点 (调试用)

```bash
# 1. 启动桥接节点
ros2 run websocket_bridge bridge_node

# 2. 启动总线舵机驱动
ros2 run servo_hardware bus_port_driver --ros-args \
  -p port:=/dev/ttyAMA0 \
  -p zl_servo_ids:="[1,2]" \
  -p lx_servo_ids:="[1,2]"

# 2.1 启动总线协议路由
ros2 run servo_hardware bus_protocol_router --ros-args \
  -p bus_map_file:=$PWD/src/websocket/config/bus_servo_map.json

# 3. 启动PCA舵机驱动
ros2 run servo_hardware pca_servo_driver

# 4. 启动 Isaac 桥接节点（可选，full_system 默认已启用）
ros2 run websocket_bridge isaac_bridge_node
```

---

## ROS 2 话题

### 发布话题

| 话题             | 消息类型       | 说明         |
| ---------------- | -------------- | ------------ |
| `/servo/command` | `ServoCommand` | 舵机控制命令 |
| `/sim/servo_state` | `ServoState` | 仿真侧状态反馈 |

### 订阅话题

| 话题           | 消息类型     | 说明         |
| -------------- | ------------ | ------------ |
| `/servo/state` | `ServoState` | 舵机状态反馈 |
| `/sim/servo_command` | `ServoCommand` | 仿真侧控制命令 |

### 查看话题

```bash
# 列出所有话题
ros2 topic list

# 查看舵机命令
ros2 topic echo /servo/command

# 查看舵机状态
ros2 topic echo /servo/state

# 发送测试命令
ros2 topic pub /servo/command servo_msgs/msg/ServoCommand \
  "{servo_type: 'bus', servo_id: 1, position: 1500, speed: 100}"
```


## 硬件配置

### 树莓派多串口配置

系统支持4个独立串口（ttyAMA0-3）同时控制14个舵机。

#### 1. 启用硬件串口

编辑 `/boot/firmware/config.txt`:

```ini
# 启用硬件串口
enable_uart=1
dtoverlay=disable-bt

# 启用额外的串口（如需）
dtoverlay=uart2
dtoverlay=uart3
dtoverlay=uart4
```

#### 2. 验证串口设备

重启后验证所有串口:

```bash
ls -l /dev/ttyAMA*
# 应该显示:
# crw-rw---- 1 root dialout 204, 64 ... /dev/ttyAMA0
# crw-rw---- 1 root dialout 204, 65 ... /dev/ttyAMA1
# crw-rw---- 1 root dialout 204, 66 ... /dev/ttyAMA2
# crw-rw---- 1 root dialout 204, 67 ... /dev/ttyAMA3
```

#### 3. 舵机ID映射

根据 `src/websocket/config/bus_servo_map.json` 配置实际ID映射：

**注意**:
- Web客户端只需发送舵机ID，系统自动路由到正确的串口
- ID映射配置在 `src/websocket/config/bus_servo_map.json`
- 支持非连续ID分配（如[3, 7, 9, 10, 11]）

### I2C配置

```bash
# 启用I2C
sudo raspi-config
# 选择: Interfacing Options → I2C → Enable

# 验证I2C设备
i2cdetect -y 1
# 应该在0x40位置看到PCA9685
```

### 权限配置

```bash
# 添加用户到dialout和i2c组
sudo usermod -a -G dialout $USER
sudo usermod -a -G i2c $USER

# 重新登录生效
```

---

## 调试方法

### 启用调试日志

```bash
# Launch文件启动时启用
ros2 launch websocket_bridge full_system.launch.py debug:=true

# 或修改launch文件默认值
# full_system.launch.py: default_value='true'
```

### 验证多串口系统

```bash
# 1. 查看所有驱动节点
ros2 node list | rg "bus_protocol_router|bus_port_driver"
# 应该看到:
# /bus_protocol_router
# /bus_port_driver_0
# /bus_port_driver_1
# /bus_port_driver_2
# /bus_port_driver_3

# 2. 检查节点参数（验证ID配置）
ros2 param get bus_port_driver_1 zl_servo_ids
# 应该返回: Integer values are: [3, 7, 9, 10, 11]

ros2 param get bus_port_driver_0 lx_servo_ids
# 应该返回: Integer values are: [1, 2]

# 3. 查看节点信息
ros2 node info /bus_protocol_router
# 查看订阅的话题和参数

# 4. 测试特定ID路由
# 测试ID=7（应该路由到ttyAMA1）
ros2 topic pub --once /servo/command servo_msgs/msg/ServoCommand \
  '{servo_type: "bus", servo_id: 7, position: 90, speed: 100}'

# 测试ID=1（应该路由到ttyAMA0）
ros2 topic pub --once /servo/command servo_msgs/msg/ServoCommand \
  '{servo_type: "bus", servo_id: 1, position: 90, speed: 100}'

# 5. 观察调试日志（debug模式）
# 应该看到类似:
# [bus_protocol_router] [INFO] [route] id=7 -> port=/dev/ttyAMA1 protocol=lx source=cache
# [bus_port_driver_1] [INFO] [Send/lx] ID=7 POS=500 SPEED=100
```

### 查看日志

```bash
# 实时查看ROS日志
ros2 run rqt_console rqt_console

# 查看特定节点日志
ros2 node info /websocket_ros2_bridge

# 查看话题频率
ros2 topic hz /servo/command

# 查看话题内容
ros2 topic echo /servo/command
```

### 测试WebSocket连接

```bash
# 使用websocat (安装: cargo install websocat)
websocat ws://192.168.31.35:9105

# 连接后发送注册消息
{"type":"register","name":"test-client"}

# 发送舵机命令
{"type":"servo_control","servo_type":"bus","servo_id":1,"position":1500,"speed":100}
```

### 测试舵机硬件

```bash
# 测试总线舵机
bash scripts/test_bus_servo.sh

# 测试PCA舵机
bash scripts/test_pca_servo.sh
```

---

## Docker配置

### docker-compose.yaml

使用 profile 控制不同运行模式：

```bash
# 开发模式（自动启动 init.sh）
docker compose --profile dev up -d ros2_servo_dev

# 生产模式（自动重启 + 健康检查）
docker compose --profile production up -d ros2_servo_prod

# 手动调试容器（只进入 bash）
docker compose --profile manual up -d ros2_servo

# 如果是从旧版本升级，先清理 orphan 容器
docker compose --profile dev --profile production --profile manual down --remove-orphans
```

### 生产模式部署

```bash
# 使用生产配置启动
docker compose --profile production up -d ros2_servo_prod

# 生产模式会自动运行full_system.launch.py
```

---

## 文件结构

```
ros/
├── src/
│   ├── servo_msgs/              # 自定义消息类型
│   │   ├── msg/
│   │   │   ├── ServoCommand.msg
│   │   │   └── ServoState.msg
│   │   └── package.xml
│   │
│   ├── servo_hardware/          # 舵机硬件驱动
│   │   ├── bus_servo.py         # 总线舵机驱动（支持ID过滤）
│   │   ├── pca_servo.py         # PCA9685驱动
│   │   ├── bus_port_driver      # 串口驱动节点（按端口）
│   │   ├── bus_protocol_router  # 协议识别与统一路由
│   │   ├── pca_servo_driver     # PCA舵机节点
│   │   └── package.xml
│   │
│   └── websocket_bridge/        # WebSocket桥接
│       ├── bridge_node.py       # ROS2桥接节点
│       ├── ws_server.py         # WebSocket服务器
│       ├── websocket_handler.py # 消息处理器
│       ├── config/              # 配置文件
│       │   ├── std_web2ros_stream.json
│       │   ├── std_ros2web_stream.json
│       │   └── bus_servo_map.json  # 舵机ID映射配置
│       ├── launch/
│       │   ├── full_system.launch.py
│       │   └── websocket_bus_servo.launch.py
│       └── package.xml
├── scripts/
│   ├── init.sh                  # 初始化脚本
│   ├── test_bus_servo.sh        # 总线舵机测试
│   └── test_pca_servo.sh        # PCA舵机测试
│
├── docker-compose.yaml          # Docker配置
├── Dockerfile                   # Docker镜像
└── README.md                    # 本文档
```



## 贡献指南

**开发规范**:
- 代码遵循PEP 8风格
- 所有注释和文档使用简体中文
- 提交前运行测试脚本
- 更新相关文档

**提交规范**:
```bash
# 格式
<type>(<scope>): <subject>

# 示例
feat(multi-serial): 支持4个独立串口同时控制
fix(websocket): 修复话题命名空间不匹配问题
docs(readme): 更新多串口系统说明
```

---

## 许可证

BSD-2.0

## 维护者

chenpeel <chenpeel@foxmail.com>
