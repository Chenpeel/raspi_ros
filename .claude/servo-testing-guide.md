# 舵机驱动测试指南

## 测试概述

本指南提供舵机驱动节点的完整测试流程，包括环境准备、硬件配置、功能测试和故障排查。

## 测试环境准备

### 1. 安装Python依赖

```bash
# 使用ROS环境的Python
~/.miniconda3/envs/ros/bin/pip install pyserial smbus2

# 或者激活conda环境后安装
conda activate ros
pip install pyserial smbus2

# 验证安装
python -c "import serial; print('pyserial version:', serial.__version__)"
python -c "import smbus2; print('smbus2 installed successfully')"
```

### 2. 构建ROS 2包

```bash
# 进入工作空间
cd /home/chenpeel/work/repo/jiyuan/ros

# 构建servo_hardware包
colcon build --packages-select servo_hardware

# 构建成功后，source环境
source install/setup.bash

# 验证节点是否可用
ros2 pkg executables servo_hardware
# 应该看到:
# servo_hardware bus_servo_driver
# servo_hardware pca_servo_driver
```

### 3. 硬件权限配置

#### 总线舵机（串口）权限

```bash
# 查看当前用户组
groups

# 添加用户到dialout组（串口访问权限）
sudo usermod -a -G dialout $USER

# 查看串口设备
ls -l /dev/ttyAMA* /dev/ttyUSB*

# 临时修改串口权限（仅用于测试）
sudo chmod 666 /dev/ttyAMA0

# 重启后权限生效（推荐）
sudo reboot
```

#### PCA9685（I2C）权限

```bash
# 启用I2C接口（树莓派）
sudo raspi-config
# 选择: Interface Options → I2C → Enable

# 或者编辑配置文件
sudo nano /boot/config.txt
# 添加: dtparam=i2c_arm=on

# 重启后生效
sudo reboot

# 验证I2C设备
ls -l /dev/i2c-*

# 扫描I2C设备（应该看到0x40地址）
i2cdetect -y 1

# 添加用户到i2c组
sudo usermod -a -G i2c $USER
```

## 测试流程

### 阶段1: 无硬件验证（基础检查）

#### 1.1 检查节点能否启动

```bash
# 测试总线舵机驱动启动（会报串口错误，正常）
ros2 run servo_hardware bus_servo_driver
# 按Ctrl+C退出

# 测试PCA驱动启动（会报I2C错误，正常）
ros2 run servo_hardware pca_servo_driver
# 按Ctrl+C退出
```

**预期结果**:
- 节点能够启动
- 看到"串口初始化失败"或"无法连接PCA9685"错误信息
- 这是正常的，因为没有连接硬件

#### 1.2 检查话题是否创建

```bash
# 在终端1启动节点（以bus_servo为例）
ros2 run servo_hardware bus_servo_driver

# 在终端2检查话题
ros2 topic list | grep servo
# 应该看到:
# /servo/command
# /servo/state

# 查看话题详细信息
ros2 topic info /servo/command
ros2 topic info /servo/state
```

#### 1.3 测试命令消息格式

```bash
# 发布测试命令（节点会尝试处理但硬件会失败）
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"bus\",\"servo_id\":1,\"position\":90,\"speed\":100}"'

# 观察节点日志输出，应该看到命令解析过程
```

### 阶段2: 总线舵机硬件测试

#### 2.1 硬件连接检查

```bash
# 检查串口设备是否存在
ls -l /dev/ttyAMA0  # 或 /dev/ttyUSB0

# 测试串口通信（发送AT命令）
echo "test" > /dev/ttyAMA0

# 使用minicom测试串口（可选）
sudo apt install minicom
minicom -D /dev/ttyAMA0 -b 115200
# 按Ctrl+A, Q 退出
```

#### 2.2 启动总线舵机驱动

```bash
# 基础启动
ros2 run servo_hardware bus_servo_driver

# 启用调试模式（推荐首次测试时使用）
ros2 run servo_hardware bus_servo_driver \
  --ros-args \
  -p port:=/dev/ttyAMA0 \
  -p baudrate:=115200 \
  -p debug:=true \
  -p log_id:=true

# 如果使用USB转串口
ros2 run servo_hardware bus_servo_driver \
  --ros-args -p port:=/dev/ttyUSB0
```

**预期结果**:
- 节点启动成功
- 日志显示: "总线舵机驱动已启动: /dev/ttyAMA0@115200"
- 没有串口错误

#### 2.3 发送控制命令

**在新终端执行**:

```bash
# 测试1: 控制舵机到90度（中间位置）
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"bus\",\"servo_id\":1,\"position\":90,\"speed\":100}"'

# 测试2: 控制舵机到0度
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"bus\",\"servo_id\":1,\"position\":0,\"speed\":500}"'

# 测试3: 控制舵机到180度
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"bus\",\"servo_id\":1,\"position\":180,\"speed\":500}"'

# 测试4: 使用脉宽值（1500us = 90度）
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"bus\",\"servo_id\":1,\"position\":1500,\"speed\":100}"'

# 测试5: 控制多个舵机
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"bus\",\"servo_id\":2,\"position\":45,\"speed\":200}"'
```

#### 2.4 监控状态反馈

```bash
# 在新终端监听状态话题
ros2 topic echo /servo/state

# 应该看到类似输出:
# data: '{"servo_type": "bus", "servo_id": 1, "position": 90, "timestamp": {...}}'
```

#### 2.5 检查驱动日志

观察驱动节点的终端输出，应该看到:

```
[INFO] [Send] ID=1 PULSE=1500 T=100ms Frame=#001P1500T0100!
[INFO] 发送成功: Port=/dev/ttyAMA0 Bytes=18 Frame=#001P1500T0100!
```

### 阶段3: PCA9685舵机硬件测试

#### 3.1 硬件连接检查

```bash
# 安装I2C工具
sudo apt install i2c-tools

# 扫描I2C总线1（树莓派默认）
i2cdetect -y 1

# 应该看到类似输出（0x40是PCA9685默认地址）:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# ...
```

#### 3.2 启动PCA9685驱动

```bash
# 基础启动
ros2 run servo_hardware pca_servo_driver

# 启用调试模式
ros2 run servo_hardware pca_servo_driver \
  --ros-args \
  -p i2c_address:=0x40 \
  -p bus_number:=1 \
  -p frequency:=50 \
  -p debug:=true

# 自定义PWM范围（如果你的舵机不同）
ros2 run servo_hardware pca_servo_driver \
  --ros-args \
  -p min_pwm:=150 \
  -p max_pwm:=600
```

**预期结果**:
- 节点启动成功
- 日志显示: "PCA9685设备检测成功(地址: 0x40)"
- 日志显示: "PCA9685舵机驱动已启动: 地址=0x40, 频率=50Hz"

#### 3.3 发送控制命令

```bash
# 测试1: 控制通道0到90度
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"pca\",\"servo_id\":0,\"position\":90}"'

# 测试2: 控制通道0到0度
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"pca\",\"servo_id\":0,\"position\":0}"'

# 测试3: 控制通道0到180度
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"pca\",\"servo_id\":0,\"position\":180}"'

# 测试4: 使用微秒值
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"pca\",\"servo_id\":0,\"position\":1500}"'

# 测试5: 控制多个通道
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"pca\",\"servo_id\":1,\"position\":45}"'

ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"pca\",\"servo_id\":2,\"position\":135}"'
```

#### 3.4 监控状态反馈

```bash
ros2 topic echo /servo/state

# 应该看到:
# data: '{"servo_type": "pca", "servo_id": 0, "position": 90, "timestamp": {...}}'
```

### 阶段4: 同时运行两个驱动

#### 4.1 使用launch文件（推荐）

创建launch文件 `src/hardware/launch/servo_drivers.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 总线舵机驱动
        Node(
            package='servo_hardware',
            executable='bus_servo_driver',
            name='bus_servo_driver',
            parameters=[
                {'port': '/dev/ttyAMA0'},
                {'baudrate': 115200},
                {'debug': True},
                {'log_id': True}
            ],
            output='screen'
        ),

        # PCA9685驱动
        Node(
            package='servo_hardware',
            executable='pca_servo_driver',
            name='pca_servo_driver',
            parameters=[
                {'i2c_address': 0x40},
                {'bus_number': 1},
                {'debug': True}
            ],
            output='screen'
        ),
    ])
```

启动:
```bash
ros2 launch servo_hardware servo_drivers.launch.py
```

#### 4.2 手动多终端启动

```bash
# 终端1: 总线舵机
ros2 run servo_hardware bus_servo_driver --ros-args -p debug:=true

# 终端2: PCA舵机
ros2 run servo_hardware pca_servo_driver --ros-args -p debug:=true

# 终端3: 发送混合命令
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"bus\",\"servo_id\":1,\"position\":90,\"speed\":100}"'

ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"pca\",\"servo_id\":0,\"position\":90}"'

# 终端4: 监控状态
ros2 topic echo /servo/state
```

### 阶段5: 性能和压力测试

#### 5.1 命令频率测试

```bash
# 测试高频命令（10Hz）
ros2 topic pub --rate 10 /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"bus\",\"servo_id\":1,\"position\":90,\"speed\":50}"'

# 观察节点CPU占用和响应延迟
top -p $(pgrep -f bus_servo_driver)
```

#### 5.2 多舵机并发测试

创建测试脚本 `test_multi_servo.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class ServoTester(Node):
    def __init__(self):
        super().__init__('servo_tester')
        self.pub = self.create_publisher(String, '/servo/command', 10)

    def test_sweep(self, servo_type, servo_id, min_angle=0, max_angle=180, step=10):
        """舵机扫描测试"""
        for angle in range(min_angle, max_angle + 1, step):
            cmd = {
                "servo_type": servo_type,
                "servo_id": servo_id,
                "position": angle,
                "speed": 100
            }
            msg = String()
            msg.data = json.dumps(cmd)
            self.pub.publish(msg)
            self.get_logger().info(f'{servo_type} ID={servo_id} → {angle}°')
            time.sleep(0.1)

def main():
    rclpy.init()
    tester = ServoTester()

    try:
        # 测试总线舵机ID=1
        tester.get_logger().info('开始测试总线舵机...')
        tester.test_sweep('bus', 1)

        time.sleep(1)

        # 测试PCA通道0
        tester.get_logger().info('开始测试PCA舵机...')
        tester.test_sweep('pca', 0)

    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

运行:
```bash
chmod +x test_multi_servo.py
python3 test_multi_servo.py
```

## 故障排查

### 问题1: 串口打不开

**症状**:
```
[ERROR] 初始化串口失败: [Errno 13] Permission denied: '/dev/ttyAMA0'
```

**解决方案**:
```bash
# 检查用户组
groups | grep dialout

# 如果没有，添加用户到dialout组
sudo usermod -a -G dialout $USER
sudo reboot

# 或临时修改权限
sudo chmod 666 /dev/ttyAMA0
```

### 问题2: I2C设备找不到

**症状**:
```
[ERROR] 无法连接PCA9685(地址: 0x40): [Errno 2] No such file or directory
```

**解决方案**:
```bash
# 检查I2C是否启用
ls -l /dev/i2c-*

# 如果没有，启用I2C
sudo raspi-config
# Interface Options → I2C → Enable

# 检查设备是否存在
i2cdetect -y 1

# 检查内核模块
lsmod | grep i2c
```

### 问题3: 舵机不动

**症状**: 命令发送成功，但舵机没有反应

**检查清单**:

1. **电源检查**:
   ```bash
   # 舵机需要独立5V电源（不能用树莓派5V供电）
   # 检查PCA9685的VCC和V+接线
   # 检查电源电流是否足够（每个舵机约500mA）
   ```

2. **接线检查**:
   - 总线舵机: 信号线、VCC、GND正确连接
   - PCA9685: SCL、SDA、VCC、GND正确连接，舵机连接到正确通道

3. **参数检查**:
   ```bash
   # 查看实际发送的PWM值
   ros2 run servo_hardware bus_servo_driver --ros-args -p debug:=true

   # 检查PCA的PWM范围是否匹配你的舵机
   # 有些舵机需要不同的PWM范围
   ros2 run servo_hardware pca_servo_driver \
     --ros-args -p min_pwm:=150 -p max_pwm:=600
   ```

4. **舵机ID检查** (总线舵机):
   ```bash
   # 确认舵机的实际ID
   # 有些舵机出厂ID不是1
   # 尝试ID 0-10
   for id in {0..10}; do
     ros2 topic pub --once /servo/command std_msgs/String \
       "data: '{\"servo_type\":\"bus\",\"servo_id\":$id,\"position\":90,\"speed\":100}'"
     sleep 1
   done
   ```

### 问题4: 串口数据乱码

**症状**: 接收线程报告解码错误

**解决方案**:
```bash
# 检查波特率是否正确
# 常见波特率: 9600, 57600, 115200
ros2 run servo_hardware bus_servo_driver \
  --ros-args -p baudrate:=9600

# 检查串口设备是否正确
# 可能是ttyS0而不是ttyAMA0
ros2 run servo_hardware bus_servo_driver \
  --ros-args -p port:=/dev/ttyS0
```

### 问题5: I2C通信错误

**症状**:
```
[ERROR] 向I2C写入失败: [Errno 121] Remote I/O error
```

**解决方案**:
```bash
# 降低I2C速度
sudo nano /boot/config.txt
# 添加: dtparam=i2c_arm_baudrate=10000

# 检查I2C接线
# SCL → GPIO3 (Pin 5)
# SDA → GPIO2 (Pin 3)
# 确保使用上拉电阻（通常PCA9685板载）

# 检查I2C地址是否正确
i2cdetect -y 1
# 如果地址不是0x40，修改参数
ros2 run servo_hardware pca_servo_driver \
  --ros-args -p i2c_address:=0x41  # 你的实际地址
```

## 测试检查清单

### 基础功能测试
- [ ] 节点能够启动
- [ ] 话题正确创建
- [ ] 命令消息格式正确解析
- [ ] 状态消息正确发布

### 总线舵机测试
- [ ] 串口设备正确打开
- [ ] 协议帧正确生成
- [ ] 舵机响应0°命令
- [ ] 舵机响应90°命令
- [ ] 舵机响应180°命令
- [ ] 角度/脉宽自动识别正确
- [ ] 多舵机控制正常

### PCA9685测试
- [ ] I2C设备正确连接
- [ ] PCA9685初始化成功
- [ ] 单通道控制正常
- [ ] 多通道控制正常
- [ ] 角度/微秒/tick自动识别正确
- [ ] PWM输出频率正确（50Hz）

### 综合测试
- [ ] 两个驱动可同时运行
- [ ] 命令路由正确（根据servo_type）
- [ ] 高频命令响应正常（10Hz）
- [ ] 节点优雅退出（舵机复位）
- [ ] 异常恢复正常（断线重连）

## 测试日志记录

建议创建测试日志文件记录每次测试结果:

```bash
# 创建测试日志
echo "# 舵机驱动测试日志" > test-log.md
echo "测试时间: $(date)" >> test-log.md
echo "" >> test-log.md

# 记录测试结果
echo "## 总线舵机测试" >> test-log.md
echo "- 串口设备: /dev/ttyAMA0" >> test-log.md
echo "- 舵机ID: 1" >> test-log.md
echo "- 测试结果: ✅ 通过" >> test-log.md
```

## 下一步

测试通过后:
1. 创建launch文件便于启动
2. 编写单元测试（Mock硬件）
3. 集成到机器人控制系统
4. 性能调优和参数标定

---

**重要提醒**:
- 首次测试建议启用 `debug:=true` 参数查看详细日志
- 舵机需要独立电源供电，不要用树莓派5V供电
- 测试前确认舵机转动范围，避免机械损坏
- 建议从中间位置（90°）开始测试

生成时间: 2025-12-15
