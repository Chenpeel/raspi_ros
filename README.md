# ROS 2 舵机控制系统

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
cd ~/work/repo/jiyuan/ros
docker-compose build

# 2. 启动开发容器
docker-compose up -d ros2_servo

# 3. 进入容器
docker exec -it ros2_servo_control bash

# 4. 编译
source /opt/ros/jazzy/setup.bash
colcon build

# 5. 运行
bash scripts/init.sh
```

### 本地编译

```bash
# 1. 安装依赖
sudo apt update
sudo apt install -y python3-pip python3-websockets

# 2. 进入工作区
cd ~/work/repo/jiyuan/ros

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
  debug:=true \
  serial_port:=/dev/ttyAMA0 \
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
ros2 run servo_hardware bus_servo_driver

# 3. 启动PCA舵机驱动
ros2 run servo_hardware pca_servo_driver
```

---

## ROS 2 话题

### 发布话题

| 话题             | 消息类型       | 说明         |
| ---------------- | -------------- | ------------ |
| `/servo/command` | `ServoCommand` | 舵机控制命令 |

### 订阅话题

| 话题           | 消息类型     | 说明         |
| -------------- | ------------ | ------------ |
| `/servo/state` | `ServoState` | 舵机状态反馈 |

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
  "{servo_type: 'bus', servo_id: 1, position: 2048, speed: 100}"
```


## 硬件配置

### 树莓派串口配置

编辑 `/boot/firmware/config.txt`:

```ini
# 启用硬件串口
enable_uart=1
dtoverlay=disable-bt
```

重启后验证:

```bash
ls -l /dev/ttyAMA0
# 应该显示: crw-rw---- 1 root dialout 204, 64 ...
```

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

### 查看日志

```bash
# 实时查看ROS日志
ros2 run rqt_console rqt_console

# 查看特定节点日志
ros2 node info /websocket_ros2_bridge

# 查看话题频率
ros2 topic hz /servo/command
```

### 测试WebSocket连接

```bash
# 使用websocat (安装: cargo install websocat)
websocat ws://192.168.31.35:9105

# 连接后发送注册消息
{"type":"register","name":"test-client"}

# 发送舵机命令
{"type":"servo_control","servo_type":"bus","servo_id":1,"position":2048,"speed":100}
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

```yaml
services:
  ros2_servo:
    build:
      context: .
      dockerfile: Dockerfile
    container_name: ros2_servo_control
    privileged: true
    network_mode: host  # 端口自动映射 (9105)

    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0   # USB串口
      - /dev/ttyAMA0:/dev/ttyAMA0   # 树莓派硬件串口
      - /dev/i2c-1:/dev/i2c-1       # I2C总线

    volumes:
      - ./src:/root/ros_ws/src:rw
      - ./install:/root/ros_ws/install:rw

    environment:
      - ROS_DOMAIN_ID=0
      - PYTHONUNBUFFERED=1

    command: /bin/bash
```

### 生产模式部署

```bash
# 使用生产配置启动
docker-compose --profile production up -d ros2_servo_prod

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
│   │   ├── bus_servo.py         # 总线舵机驱动
│   │   ├── pca_servo.py         # PCA9685驱动
│   │   ├── bus_servo_driver     # 总线舵机节点
│   │   ├── pca_servo_driver     # PCA舵机节点
│   │   └── package.xml
│   │
│   └── websocket_bridge/        # WebSocket桥接
│       ├── bridge_node.py       # ROS2桥接节点
│       ├── ws_server.py         # WebSocket服务器
│       ├── websocket_handler.py # 消息处理器
│       ├── launch/
│       │   └── full_system.launch.py
│       └── package.xml
│
├── scripts/
│   ├── init.sh                  # 初始化脚本
│   ├── test_bus_servo.sh        # 总线舵机测试
│   └── test_pca_servo.sh        # PCA舵机测试
│
├── docker-compose.yaml          # Docker配置
├── Dockerfile                   # Docker镜像
└── README.md                    # 本文档
```

---

## 常见问题

### Q1: WebSocket端口冲突

**问题**: `[Errno 98] address already in use`

**解决**:
```bash
# 查看占用9105端口的进程
lsof -i :9105

# 杀死进程或修改端口配置
ros2 launch websocket_bridge full_system.launch.py ws_port:=9106
```

### Q2: 舵机无响应

**问题**: 命令发送成功但舵机不动

**解决**:
1. 检查串口权限: `ls -l /dev/ttyAMA0`
2. 检查串口配置: `stty -F /dev/ttyAMA0`
3. 测试舵机硬件: `bash scripts/test_bus_servo.sh`
4. 查看驱动日志: `ros2 topic echo /servo/state`

### Q3: I2C设备未找到

**问题**: `[Errno 121] Remote I/O error`

**解决**:
```bash
# 检查I2C是否启用
sudo raspi-config

# 扫描I2C设备
i2cdetect -y 1

# 检查设备权限
ls -l /dev/i2c-1
```

### Q4: Docker容器权限问题

**问题**: 编译时出现权限错误

**解决**:
```bash
# 在容器外清理
sudo rm -rf install build

# 重新进入容器编译
docker exec -it ros2_servo_control bash
colcon build
```

### Q5: 前端连接失败

**问题**: 浏览器控制台显示WebSocket连接失败

**解决**:
1. 检查ROS2服务是否运行: `ros2 node list`
2. 检查端口是否监听: `netstat -tulpn | grep 9105`
3. 检查防火墙: `sudo ufw status`
4. 检查前端配置: `WS_SERVO_URL`是否正确

---

## 性能优化

### 减少日志输出

```bash
# 关闭调试模式
ros2 launch websocket_bridge full_system.launch.py debug:=false

# 或设置ROS日志级别
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
```

### 提高舵机响应速度

修改 `bridge_node.py`:

```python
# 降低心跳间隔 (默认15秒)
self.heartbeat_interval = 5.0
```

修改 `bus_servo.py`:

```python
# 增加串口读取超时
self.serial.timeout = 0.01  # 减少到10ms
```

---

## 扩展开发

### 添加新的舵机驱动

1. 在 `src/servo_hardware/` 创建新驱动类
2. 继承基类并实现接口:
   ```python
   class MyServoDriver:
       def set_position(self, servo_id, position, speed):
           pass

       def get_position(self, servo_id):
           pass
   ```

3. 创建ROS2节点包装器
4. 在launch文件中添加节点配置

### 添加新的消息类型

1. 在 `src/servo_msgs/msg/` 创建 `.msg` 文件
2. 修改 `CMakeLists.txt` 添加消息定义
3. 重新编译: `colcon build --packages-select servo_msgs`

---

## 贡献指南

欢迎提交Issue和Pull Request！

开发规范:
- 代码遵循PEP 8风格
- 提交前运行测试脚本
- 更新相关文档

---

## 许可证

BSD-2.0

## 维护者

chenpeel <chgenpeel@foxmail.com>
