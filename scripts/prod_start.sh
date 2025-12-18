#!/bin/bash
# ============================================
# ROS 2 舵机控制系统 - 生产模式启动脚本
# 功能：使用预构建的工作空间，快速启动系统
# ============================================

set -e  # 遇到错误立即退出
set -u  # 使用未定义变量时报错

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 打印启动横幅
echo "============================================"
echo -e "${GREEN}ROS 2 舵机控制系统 - 生产模式${NC}"
echo "启动时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "工作目录: $(pwd)"
echo "============================================"
echo ""

# 1. 加载 ROS 2 环境
log_info "加载 ROS 2 环境 ($ROS_DISTRO)..."
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    log_success "ROS 2 环境加载完成"
else
    log_error "找不到 ROS 2 环境文件"
    exit 1
fi

# 2. 检查工作空间是否已构建
log_info "检查工作空间..."
if [ ! -f "/root/ros_ws/install/setup.bash" ]; then
    log_error "未找到预构建的工作空间！"
    echo ""
    echo "请选择以下方式之一："
    echo "  1. 在宿主机上运行: colcon build"
    echo "  2. 在 Dockerfile 中预构建代码"
    echo "  3. 使用 init.sh 代替本脚本（每次启动时构建）"
    echo ""
    exit 1
fi

# 3. 加载工作空间环境
log_info "加载工作空间环境..."
source /root/ros_ws/install/setup.bash
log_success "工作空间环境加载完成"

# 4. 检查关键设备
echo ""
log_info "检查硬件设备..."
devices_ok=true

check_device() {
    local device=$1
    local name=$2
    if [ -e "$device" ]; then
        log_success "  ✓ $name: $device"
    else
        log_warning "  ✗ $name: $device (未找到)"
        devices_ok=false
    fi
}

check_device "/dev/ttyUSB0" "USB串口"
check_device "/dev/ttyAMA0" "树莓派串口"
check_device "/dev/i2c-1" "I2C总线"

if [ "$devices_ok" = false ]; then
    log_warning "部分设备未找到，系统可能无法正常工作"
    log_warning "按 Ctrl+C 取消，或等待 5 秒继续启动..."
    sleep 5
fi

# 5. 显示系统信息
echo ""
log_info "系统信息："
echo "  - ROS_DISTRO: $ROS_DISTRO"
echo "  - ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-未设置}"
echo "  - 工作空间: /root/ros_ws"
echo "  - Python: $(python3 --version)"

# 6. 启动系统
echo ""
echo "============================================"
log_info "启动完整系统..."
echo "============================================"
echo ""

# 使用 exec 替换当前进程，确保信号正确传递
exec ros2 launch websocket_bridge full_system.launch.py
