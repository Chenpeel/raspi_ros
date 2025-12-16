# 总线舵机无响应诊断清单

## 当前状态分析

从你的日志看到：
```
[bus_servo_driver-2] [INFO] [bus_servo_driver]: [Send] ID=89 PULSE=1000 T=50ms Frame=#089P1000T0050!
[bridge_node-1] [INFO] [websocket_ros2_bridge]: 收到舵机状态: bus ID=89 POS=45
```

**好消息**：
- ✅ 软件通信正常（ROS节点→驱动板通信正常）
- ✅ 命令格式正确（`#089P1000T0050!`）
- ✅ 串口发送成功

**问题**：舵机没有实际运动

## 硬件诊断步骤

### 步骤1: 验证串口通信（最关键）

```bash
# 1. 检查串口设备是否正确
ls -l /dev/ttyUSB*
# 应该看到 /dev/ttyUSB0 或类似设备

# 2. 停止ROS节点（Ctrl+C），然后手动测试串口
# 安装串口调试工具
sudo apt install minicom

# 打开串口（波特率115200）
minicom -D /dev/ttyUSB0 -b 115200

# 在minicom中手动输入命令（注意大小写）：
#001P1500T0100!
# 按回车发送

# 如果舵机动了 → 硬件OK，是软件配置问题
# 如果舵机不动 → 继续检查硬件
```

### 步骤2: 检查舵机ID

**可能原因**：你的舵机ID不是89

```bash
# 方法1: 扫描所有ID（0-99）
# 创建测试脚本 scan_servo_id.sh：
for id in {0..99}; do
  echo "#$(printf "%03d" $id)P1500T0100!" > /dev/ttyUSB0
  sleep 0.5
done

# 方法2: 手动测试常见ID
echo "#001P1500T0100!" > /dev/ttyUSB0  # 测试ID=1
sleep 1
echo "#000P1500T0100!" > /dev/ttyUSB0  # 测试ID=0
sleep 1
echo "#255P1500T0100!" > /dev/ttyUSB0  # 测试ID=255（广播）
```

**提示**：
- 大多数舵机出厂默认ID是 **1** 或 **0**，不是89
- ID=255通常是广播地址，会控制所有舵机

### 步骤3: 检查物理连接

#### 3.1 供电检查（最常见问题）

舵机**必须独立供电**，不能只靠USB供电！

```
正确接线:
┌─────────────┐
│ USB转TTL板  │
│   (驱动板)  │
└─────┬───────┘
      │
  ┌───┴────┬─────┬─────┐
  │  GND   │ TXD │ RXD │
  └───┬────┴──┬──┴──┬──┘
      │       │     │
      │    ┌──┴──┐  │
      │    │舵机 │  │
      │    │信号线│  │
      │    └──┬──┘  │
      │       │     │
  ┌───┴───────┴─────┴──┐
  │   5V/6V 电源适配器  │
  │   (至少2A电流)      │
  └─────────────────────┘

关键点:
1. 舵机的VCC和GND接到独立电源（5V/6V，2A以上）
2. 舵机的信号线（黄色/白色）接到驱动板的TXD
3. 舵机的GND与驱动板的GND共地
4. USB只提供数据通信，不提供舵机电源！
```

**检查清单**：
- [ ] 舵机是否有独立5V/6V电源？
- [ ] 电源电流是否足够（至少2A）？
- [ ] 舵机GND是否与驱动板GND共地？
- [ ] 信号线是否接对（TXD→舵机信号）？

#### 3.2 接线检查

```
标准总线舵机接线（3线）：
- 红线（VCC）  → 独立5V电源正极
- 黑/棕线（GND） → 独立5V电源负极 + 驱动板GND
- 黄/白线（信号）→ 驱动板TXD

USB转TTL驱动板：
- TXD → 舵机信号线
- RXD → （可选）接舵机反馈线，单向控制可不接
- GND → 与舵机电源GND共地
- VCC → 不要接！
```

### 步骤4: 检查波特率

总线舵机常见波特率：
- 9600
- 57600
- **115200**（你当前使用的）
- 1000000

尝试不同波特率：
```bash
# 测试9600
ros2 run servo_hardware bus_servo_driver \
  --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=9600

# 测试115200（当前）
ros2 run servo_hardware bus_servo_driver \
  --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```

### 步骤5: 检查协议格式

你的舵机可能使用不同的协议格式。常见格式：

**格式1（当前使用）**：
```
#001P1500T0100!
```

**格式2（HiWonder/FeeTech）**：
```
#1P1500T100!  (无前导0)
```

**格式3（无时间参数）**：
```
#001P1500!
```

### 步骤6: 使用示波器/逻辑分析仪

如果有示波器：
```bash
# 观察TXD引脚波形
# 应该看到115200波特率的串行数据
# 每次发送命令时应该有电平变化
```

## 快速排查命令

```bash
# 1. 停止所有ROS节点
# Ctrl+C 停止launch

# 2. 测试最简单的命令（ID=1，中间位置）
echo "#001P1500T0100!" > /dev/ttyUSB0

# 3. 测试广播命令（所有舵机）
echo "#255P1500T0100!" > /dev/ttyUSB0

# 4. 如果舵机动了，说明是ID或配置问题
# 如果舵机不动，检查供电和接线
```

## 常见问题和解决方案

### Q1: 舵机ID不对
**症状**：命令发送成功，但舵机不动

**解决**：
```bash
# 使用舵机调试软件重新设置ID
# 或使用ID=255广播测试：
ros2 topic pub --once /servo/command std_msgs/String \
  'data: "{\"servo_type\":\"bus\",\"servo_id\":255,\"position\":90,\"speed\":100}"'
```

### Q2: 供电不足
**症状**：舵机抖动或完全不动

**解决**：
- 使用至少2A的5V独立电源
- 确保电源电压在4.8V-6V之间
- 不要用USB供电给舵机

### Q3: 接线错误
**症状**：无任何反应

**解决**：
```
正确接法（再次确认）：
USB转TTL的TXD → 舵机信号线（黄/白）
USB转TTL的GND → 舵机GND（黑/棕）+ 电源GND
独立电源5V    → 舵机VCC（红）
独立电源GND   → 舵机GND（黑/棕）+ USB转TTL GND
```

### Q4: 驱动板方向错误
**症状**：发送命令但舵机无响应

**可能原因**：TXD和RXD接反了

**解决**：
```bash
# 如果当前TXD接舵机信号线不行
# 尝试改用RXD接舵机信号线
# 然后修改代码中的串口方向
```

## 调试工具推荐

### 1. 串口助手（Linux）
```bash
# minicom
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200

# screen
screen /dev/ttyUSB0 115200

# cutecom（图形界面）
sudo apt install cutecom
cutecom
```

### 2. 监听串口发送内容
```bash
# 使用strace监听
strace -e write -p $(pgrep bus_servo) 2>&1 | grep ttyUSB

# 或使用interceptty（串口嗅探）
sudo apt install interceptty
interceptty /dev/ttyUSB0 /tmp/vserial0 /tmp/vserial1
# 然后连接/tmp/vserial0进行通信，监听/tmp/vserial1
```

### 3. 创建简单测试脚本

```python
#!/usr/bin/env python3
"""简单的总线舵机测试脚本"""

import serial
import time

# 打开串口
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def send_command(servo_id, position, speed=100):
    """发送舵机命令"""
    # 角度转脉宽
    pulse = int(500 + (position / 180.0) * 2000)

    # 格式化命令
    frame = f"#{servo_id:03d}P{pulse:04d}T{speed:04d}!"

    print(f"发送: {frame}")
    ser.write(frame.encode('utf-8'))
    time.sleep(0.05)

# 测试不同ID
print("开始扫描舵机ID...")
for test_id in [0, 1, 2, 89, 255]:
    print(f"\n测试ID={test_id}:")
    send_command(test_id, 90, 100)
    time.sleep(1)
    input(f"ID={test_id}的舵机有没有动？按回车继续...")

ser.close()
print("\n测试完成！")
```

运行：
```bash
chmod +x test_servo.py
python3 test_servo.py
```

## 我的建议

基于你的情况，**最可能的问题**：

1. **舵机ID不是89**（90%可能性）
   - 先用ID=1或ID=255测试

2. **供电不足**（80%可能性）
   - 确认舵机有独立电源

3. **接线错误**（50%可能性）
   - TXD接舵机信号线
   - GND共地

**立即尝试**：
```bash
# 停止ROS节点，直接测试ID=1
echo "#001P1500T0100!" > /dev/ttyUSB0
# 舵机动了吗？

# 测试ID=255（广播）
echo "#255P1500T0100!" > /dev/ttyUSB0
# 舵机动了吗？
```

如果这两个命令舵机能动，说明硬件OK，只需要修改软件配置的ID即可。

---

生成时间: 2025-12-15
