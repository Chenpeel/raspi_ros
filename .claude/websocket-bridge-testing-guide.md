# WebSocket到舵机驱动完整测试指南

## 系统架构

```
WebSocket客户端 (Web/App)
       ↓ WebSocket协议
WebSocket服务器 (bridge_node)
       ↓ ROS 2话题 (/servo/command)
舵机驱动节点 (bus_servo_driver / pca_servo_driver)
       ↓ 硬件协议 (串口/I2C)
舵机硬件
```

## 数据流

### 下行 (控制命令)
1. WebSocket客户端发送JSON: `{"b": -1, "c": 1, "p": 90, "s": 100}` 或 `{"servo_type": "bus", "servo_id": 1, "position": 90, "speed": 100}`
2. bridge_node接收并解析为标准格式
3. bridge_node发布到 `/servo/command` 话题
4. 舵机驱动节点订阅并接收命令
5. 舵机驱动节点转换为硬件协议并发送

### 上行 (状态反馈)
1. 舵机驱动节点发布状态到 `/servo/state` 话题
2. bridge_node订阅并接收状态
3. bridge_node广播到所有WebSocket客户端

## 测试准备

### 1. 安装依赖

```bash
# Python依赖
~/.miniconda3/envs/ros/bin/pip install pyserial smbus2 websockets

# 或激活环境后安装
conda activate ros
pip install pyserial smbus2 websockets
```

### 2. 构建包

```bash
cd /home/chenpeel/work/repo/jiyuan/ros

# 构建所有相关包
colcon build --packages-select websocket_bridge servo_hardware

# source环境
source install/setup.bash

# 验证可执行文件
ros2 pkg executables websocket_bridge
# 应该看到:
# websocket_bridge bridge_node
# websocket_bridge ws_server

ros2 pkg executables servo_hardware
# 应该看到:
# servo_hardware bus_servo_driver
# servo_hardware pca_servo_driver
```

### 3. 硬件权限配置

参考舵机驱动测试指南 `.claude/servo-testing-guide.md` 中的权限配置部分。

## 测试场景

### 场景1: 完整系统测试（推荐）

启动完整系统（WebSocket + 总线舵机 + PCA舵机）:

```bash
ros2 launch websocket_bridge full_system.launch.py
```

**自定义参数**:
```bash
ros2 launch websocket_bridge full_system.launch.py \
  ws_host:=0.0.0.0 \
  ws_port:=9102 \
  device_id:=robot \
  serial_port:=/dev/ttyAMA0 \
  baudrate:=115200 \
  i2c_address:=64 \
  i2c_bus:=1 \
  debug:=true
```

### 场景2: WebSocket + 总线舵机

```bash
ros2 launch websocket_bridge websocket_bus_servo.launch.py
```

### 场景3: 手动逐个启动（调试模式）

**终端1: WebSocket桥接节点**
```bash
ros2 run websocket_bridge bridge_node
```

**终端2: 总线舵机驱动**
```bash
ros2 run servo_hardware bus_servo_driver \
  --ros-args -p debug:=true -p log_id:=true
```

**终端3: PCA舵机驱动**
```bash
ros2 run servo_hardware pca_servo_driver \
  --ros-args -p debug:=true
```

## WebSocket客户端测试

### 方法1: 使用Python客户端测试

创建测试脚本 `test_ws_client.py`:

```python
#!/usr/bin/env python3
"""
WebSocket客户端测试工具
"""

import asyncio
import json
import websockets


async def test_servo_control():
    """测试舵机控制"""
    uri = "ws://localhost:9102"

    async with websockets.connect(uri) as websocket:
        print(f"已连接到: {uri}")

        # 测试1: 发送BCP格式（总线舵机）
        print("\n[测试1] 发送BCP格式命令...")
        bcp_cmd = {
            "b": -1,  # 总线舵机标识
            "c": 1,   # 舵机ID=1
            "p": 90,  # 位置90度
            "s": 100  # 速度100ms
        }
        await websocket.send(json.dumps(bcp_cmd))
        response = await websocket.recv()
        print(f"收到响应: {response}")

        await asyncio.sleep(1)

        # 测试2: 发送完整格式（总线舵机）
        print("\n[测试2] 发送完整格式命令（总线舵机）...")
        full_cmd = {
            "servo_type": "bus",
            "servo_id": 1,
            "position": 0,
            "speed": 500
        }
        await websocket.send(json.dumps(full_cmd))
        response = await websocket.recv()
        print(f"收到响应: {response}")

        await asyncio.sleep(2)

        # 测试3: PCA舵机
        print("\n[测试3] 发送PCA舵机命令...")
        pca_cmd = {
            "b": 0,   # PCA标识
            "c": 0,   # 通道0
            "p": 90   # 位置90度
        }
        await websocket.send(json.dumps(pca_cmd))
        response = await websocket.recv()
        print(f"收到响应: {response}")

        await asyncio.sleep(1)

        # 测试4: 状态查询
        print("\n[测试4] 发送状态查询...")
        status_query = {
            "type": "status_query"
        }
        await websocket.send(json.dumps(status_query))
        response = await websocket.recv()
        print(f"收到状态: {response}")

        # 测试5: 监听状态反馈
        print("\n[测试5] 监听舵机状态反馈（5秒）...")
        try:
            for _ in range(5):
                response = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                print(f"状态更新: {response}")
        except asyncio.TimeoutError:
            print("无新状态更新")


if __name__ == "__main__":
    asyncio.run(test_servo_control())
```

运行测试:
```bash
chmod +x test_ws_client.py
python3 test_ws_client.py
```

### 方法2: 使用wscat（命令行工具）

```bash
# 安装wscat
npm install -g wscat

# 连接到WebSocket服务器
wscat -c ws://localhost:9102

# 发送命令（在wscat交互界面）
# 总线舵机 - BCP格式
{"b": -1, "c": 1, "p": 90, "s": 100}

# 总线舵机 - 完整格式
{"servo_type": "bus", "servo_id": 1, "position": 90, "speed": 100}

# PCA舵机
{"servo_type": "pca", "servo_id": 0, "position": 90}

# 状态查询
{"type": "status_query"}
```

### 方法3: 使用Web浏览器测试

创建 `test_ws.html`:

```html
<!DOCTYPE html>
<html>
<head>
    <title>WebSocket舵机控制测试</title>
    <meta charset="utf-8">
    <style>
        body { font-family: monospace; padding: 20px; }
        button { margin: 5px; padding: 10px; font-size: 14px; }
        #log {
            border: 1px solid #ccc;
            padding: 10px;
            height: 400px;
            overflow-y: scroll;
            background: #f5f5f5;
        }
        .sent { color: blue; }
        .received { color: green; }
        .error { color: red; }
    </style>
</head>
<body>
    <h1>WebSocket舵机控制测试</h1>

    <div>
        <input type="text" id="wsUrl" value="ws://localhost:9102" style="width: 300px;">
        <button onclick="connect()">连接</button>
        <button onclick="disconnect()">断开</button>
        <span id="status">未连接</span>
    </div>

    <h2>控制面板</h2>

    <div>
        <h3>总线舵机控制</h3>
        ID: <input type="number" id="busId" value="1" min="0" max="255" style="width: 60px;">
        角度: <input type="range" id="busAngle" min="0" max="180" value="90" oninput="document.getElementById('busAngleVal').innerText=this.value">
        <span id="busAngleVal">90</span>°
        速度: <input type="number" id="busSpeed" value="100" min="10" max="1000" style="width: 80px;"> ms
        <button onclick="sendBusServo()">发送</button>
    </div>

    <div>
        <h3>PCA舵机控制</h3>
        通道: <input type="number" id="pcaChannel" value="0" min="0" max="15" style="width: 60px;">
        角度: <input type="range" id="pcaAngle" min="0" max="180" value="90" oninput="document.getElementById('pcaAngleVal').innerText=this.value">
        <span id="pcaAngleVal">90</span>°
        <button onclick="sendPcaServo()">发送</button>
    </div>

    <div>
        <h3>其他</h3>
        <button onclick="sendStatusQuery()">状态查询</button>
        <button onclick="clearLog()">清空日志</button>
    </div>

    <h2>通信日志</h2>
    <div id="log"></div>

    <script>
        let ws = null;

        function log(message, className = '') {
            const logDiv = document.getElementById('log');
            const timestamp = new Date().toLocaleTimeString();
            const entry = document.createElement('div');
            entry.className = className;
            entry.innerText = `[${timestamp}] ${message}`;
            logDiv.appendChild(entry);
            logDiv.scrollTop = logDiv.scrollHeight;
        }

        function connect() {
            const url = document.getElementById('wsUrl').value;
            ws = new WebSocket(url);

            ws.onopen = () => {
                document.getElementById('status').innerText = '已连接';
                document.getElementById('status').style.color = 'green';
                log('WebSocket已连接', 'received');
            };

            ws.onclose = () => {
                document.getElementById('status').innerText = '已断开';
                document.getElementById('status').style.color = 'red';
                log('WebSocket已断开', 'error');
            };

            ws.onerror = (error) => {
                log('WebSocket错误: ' + error, 'error');
            };

            ws.onmessage = (event) => {
                log('← 收到: ' + event.data, 'received');
            };
        }

        function disconnect() {
            if (ws) {
                ws.close();
                ws = null;
            }
        }

        function send(data) {
            if (!ws || ws.readyState !== WebSocket.OPEN) {
                log('未连接到WebSocket服务器', 'error');
                return;
            }
            const jsonStr = JSON.stringify(data);
            ws.send(jsonStr);
            log('→ 发送: ' + jsonStr, 'sent');
        }

        function sendBusServo() {
            const cmd = {
                servo_type: "bus",
                servo_id: parseInt(document.getElementById('busId').value),
                position: parseInt(document.getElementById('busAngle').value),
                speed: parseInt(document.getElementById('busSpeed').value)
            };
            send(cmd);
        }

        function sendPcaServo() {
            const cmd = {
                servo_type: "pca",
                servo_id: parseInt(document.getElementById('pcaChannel').value),
                position: parseInt(document.getElementById('pcaAngle').value)
            };
            send(cmd);
        }

        function sendStatusQuery() {
            send({ type: "status_query" });
        }

        function clearLog() {
            document.getElementById('log').innerHTML = '';
        }

        // 页面加载后自动连接
        window.onload = () => {
            connect();
        };
    </script>
</body>
</html>
```

在浏览器中打开 `test_ws.html` 进行测试。

## ROS 2话题监控

### 监控舵机命令

```bash
# 查看所有舵机命令
ros2 topic echo /servo/command

# 查看命令发送频率
ros2 topic hz /servo/command
```

### 监控舵机状态

```bash
# 查看所有状态反馈
ros2 topic echo /servo/state

# 查看状态更新频率
ros2 topic hz /servo/state
```

### 手动发送ROS命令测试

```bash
# 发送总线舵机命令
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"bus\",\"servo_id\":1,\"position\":90,\"speed\":100}"'

# 发送PCA舵机命令
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"pca\",\"servo_id\":0,\"position\":90}"'
```

## 测试检查清单

### 基础连接测试
- [ ] WebSocket服务器成功启动（端口9102）
- [ ] 舵机驱动节点成功启动
- [ ] 桥接节点成功启动
- [ ] WebSocket客户端能够连接

### 总线舵机测试
- [ ] BCP格式命令正确解析
- [ ] 完整格式命令正确解析
- [ ] 命令正确发布到 `/servo/command`
- [ ] 舵机驱动正确接收并执行
- [ ] 舵机实际运动正常
- [ ] 状态反馈正确发布到 `/servo/state`
- [ ] 状态正确广播到WebSocket客户端

### PCA舵机测试
- [ ] BCP格式命令正确解析
- [ ] 完整格式命令正确解析
- [ ] 命令正确路由到PCA驱动
- [ ] PCA驱动正确接收并执行
- [ ] 舵机实际运动正常
- [ ] 状态反馈正确

### 混合测试
- [ ] 可同时控制总线舵机和PCA舵机
- [ ] 命令路由正确（根据servo_type）
- [ ] 两种舵机状态反馈互不干扰

### 异常处理测试
- [ ] 非法命令格式正确拒绝
- [ ] 错误信息正确返回给WebSocket客户端
- [ ] 硬件断开后能正确报错
- [ ] 硬件重连后能恢复工作

### 性能测试
- [ ] 命令响应延迟 < 100ms
- [ ] 高频命令（10Hz）正常处理
- [ ] 多客户端同时连接正常
- [ ] 长时间运行稳定

## 故障排查

### 问题1: WebSocket连接失败

**症状**:
```
WebSocket connection failed: Connection refused
```

**检查**:
```bash
# 检查bridge_node是否运行
ros2 node list | grep bridge

# 检查端口是否被占用
netstat -tuln | grep 9102

# 检查防火墙
sudo ufw status
```

### 问题2: 命令发送无响应

**症状**: WebSocket发送命令后没有舵机动作

**检查步骤**:

1. **检查话题连接**:
```bash
# 查看bridge_node的话题
ros2 node info /websocket_ros2_bridge

# 查看舵机驱动的话题
ros2 node info /bus_servo_driver
ros2 node info /pca_servo_driver
```

2. **监控话题消息**:
```bash
# 终端1: 监控命令话题
ros2 topic echo /servo/command

# 终端2: 通过WebSocket发送命令
# 看是否有消息输出
```

3. **检查消息格式**:
```bash
# 查看实际发送的消息
# 在bridge_node日志中应该看到调试信息（如果启用了debug）
```

### 问题3: 状态反馈未收到

**检查**:
```bash
# 监控状态话题
ros2 topic echo /servo/state

# 如果有消息，说明驱动正常，检查bridge_node
# 如果无消息，说明驱动未发布状态
```

### 问题4: launch文件启动失败

**症状**:
```
Package 'websocket_bridge' not found
```

**解决**:
```bash
# 重新构建
colcon build --packages-select websocket_bridge

# 重新source
source install/setup.bash

# 验证
ros2 pkg list | grep websocket
```

## 测试报告模板

```markdown
# WebSocket舵机控制测试报告

## 测试环境
- 测试时间: YYYY-MM-DD
- 系统版本: Ubuntu 22.04
- ROS版本: ROS 2 Jazzy
- Python版本: 3.10

## 测试配置
- WebSocket地址: ws://localhost:9102
- 总线舵机端口: /dev/ttyAMA0
- PCA9685地址: 0x40

## 测试结果

### 1. 基础连接
- ✅ WebSocket连接正常
- ✅ ROS节点启动正常
- ✅ 话题通信正常

### 2. 总线舵机
- ✅ BCP格式解析正确
- ✅ 完整格式解析正确
- ✅ 舵机运动正常
- ✅ 状态反馈正常

### 3. PCA舵机
- ✅ 命令解析正确
- ✅ 舵机运动正常
- ✅ 状态反馈正常

### 4. 性能测试
- 命令延迟: ~50ms
- 高频测试: 10Hz稳定
- 多客户端: 3个并发正常

## 发现的问题
1. [问题描述]
   - 解决方案: [方案]

## 建议
1. [改进建议]
```

---

## 下一步

测试通过后:
1. 部署到实际机器人
2. 集成到控制面板/App
3. 添加更多功能（如动作序列、轨迹规划）
4. 性能优化

---

**重要提醒**:
- 测试前确保硬件连接正确
- 首次测试建议启用debug模式
- 建议从单个舵机开始测试
- 记录所有测试结果和问题

生成时间: 2025-12-15
