# Python环境问题诊断和修复指南

## 问题1: Python版本不匹配

### 症状
```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/jazzy/lib/python3.12/site-packages/_rclpy_pybind11.cpython-313-x86_64-linux-gnu.so' isn't present on the system.
```

### 原因
- ROS 2 Jazzy使用Python 3.12编译
- 当前系统默认Python是3.13
- Python C扩展无法跨版本使用

### 解决方案：使用系统Python 3.12

```bash
# 1. 检查系统Python版本
python3 --version
python3.12 --version

# 2. 如果没有Python 3.12，安装它
sudo apt install python3.12 python3.12-dev python3.12-venv

# 3. 使用Python 3.12运行ROS节点
# 方法1: 修改shebang（推荐）
# 方法2: 使用python3.12直接调用（下面会说明）
```

## 问题2: 缺少websockets模块

### 症状
```
ModuleNotFoundError: No module named 'websockets'
```

### 解决方案

```bash
# 安装websockets到系统Python 3.12
python3.12 -m pip install websockets pyserial smbus2

# 如果使用conda/虚拟环境，确保使用正确的Python版本
conda activate ros  # 如果你有ros环境
python --version    # 应该是3.12.x
pip install websockets pyserial smbus2
```

## 快速修复步骤

### 步骤1: 安装Python 3.12和依赖

```bash
# 安装Python 3.12
sudo apt update
sudo apt install python3.12 python3.12-dev python3.12-venv python3-pip

# 安装Python依赖到3.12
python3.12 -m pip install --user websockets pyserial smbus2
```

### 步骤2: 验证安装

```bash
# 验证Python版本
python3.12 --version
# 应该输出: Python 3.12.x

# 验证依赖
python3.12 -c "import websockets; print('websockets OK')"
python3.12 -c "import serial; print('pyserial OK')"
python3.12 -c "import smbus2; print('smbus2 OK')"
python3.12 -c "import rclpy; print('rclpy OK')"
```

### 步骤3: 使用正确的Python运行

#### 方法A: 临时修改环境变量（推荐）

```bash
# 设置ROS使用Python 3.12
export ROS_PYTHON_VERSION=3.12
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH

# 然后正常启动
source install/setup.zsh
ros2 launch websocket_bridge full_system.launch.py
```

#### 方法B: 修改launch文件指定Python解释器

编辑launch文件，添加`executable_cmd`参数：

```python
# full_system.launch.py
bridge_node = Node(
    package='websocket_bridge',
    executable='bridge_node',
    name='websocket_ros2_bridge',
    output='screen',
    executable_cmd=['/usr/bin/python3.12'],  # 指定Python 3.12
    # ...
)
```

#### 方法C: 创建Python 3.12虚拟环境（最稳定）

```bash
# 创建虚拟环境
python3.12 -m venv ~/ros_venv

# 激活虚拟环境
source ~/ros_venv/bin/activate

# 安装依赖
pip install websockets pyserial smbus2

# 设置ROS环境
source /opt/ros/jazzy/setup.bash
source ~/work/repo/jiyuan/ros/install/setup.bash

# 运行launch文件
ros2 launch websocket_bridge full_system.launch.py
```

## 推荐的工作流程

### 创建启动脚本

创建 `~/work/repo/jiyuan/ros/start_servo_system.sh`:

```bash
#!/bin/bash

# 激活Python 3.12虚拟环境（如果使用）
# source ~/ros_venv/bin/activate

# 或者直接设置Python版本
export ROS_PYTHON_VERSION=3.12

# Source ROS环境
source /opt/ros/jazzy/setup.bash
source ~/work/repo/jiyuan/ros/install/setup.bash

# 检查依赖
echo "检查Python版本..."
python3.12 --version

echo "检查依赖模块..."
python3.12 -c "import websockets; import serial; import smbus2; import rclpy; print('所有依赖OK')" || {
    echo "依赖检查失败，请安装依赖:"
    echo "python3.12 -m pip install --user websockets pyserial smbus2"
    exit 1
}

# 启动系统
echo "启动完整舵机控制系统..."
ros2 launch websocket_bridge full_system.launch.py \
    serial_port:=/dev/ttyUSB0 \
    debug:=true
```

然后运行：
```bash
chmod +x ~/work/repo/jiyuan/ros/start_servo_system.sh
~/work/repo/jiyuan/ros/start_servo_system.sh
```

## 验证和测试

### 1. 检查Python环境

```bash
# 检查当前Python
python3 --version
which python3

# 检查ROS Python
python3.12 -c "import rclpy; print(rclpy.__file__)"
```

### 2. 手动启动单个节点测试

```bash
# 设置环境
export ROS_PYTHON_VERSION=3.12
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 测试bridge_node
python3.12 $(which ros2) run websocket_bridge bridge_node

# 测试bus_servo_driver
python3.12 $(which ros2) run servo_hardware bus_servo_driver
```

### 3. 检查依赖安装位置

```bash
# 查看websockets安装位置
python3.12 -c "import websockets; print(websockets.__file__)"

# 查看rclpy安装位置
python3.12 -c "import rclpy; print(rclpy.__file__)"
```

## 常见问题

### Q1: pip install失败
```bash
# 升级pip
python3.12 -m pip install --upgrade pip

# 使用--user安装（无需sudo）
python3.12 -m pip install --user websockets pyserial smbus2
```

### Q2: 仍然报Python版本错误
```bash
# 清理构建缓存
cd ~/work/repo/jiyuan/ros
rm -rf build/ install/ log/

# 重新构建
colcon build --packages-select websocket_bridge servo_hardware

# Source新环境
source install/setup.bash
```

### Q3: 找不到python3.12命令
```bash
# Ubuntu 22.04/24.04安装Python 3.12
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.12 python3.12-dev python3.12-venv
```

## 下一步

修复后测试：
```bash
# 设置环境
export ROS_PYTHON_VERSION=3.12
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 启动完整系统
ros2 launch websocket_bridge full_system.launch.py
```

---

生成时间: 2025-12-15
