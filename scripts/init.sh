#!/bin/bash
# ============================================
# ROS 2 舵机控制系统 - 初始化脚本
# 功能：构建工作空间并启动系统
# 适用场景：开发模式、首次部署、确保代码最新
# ============================================

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[⚠]${NC} $1"
}

log_error() {
    echo -e "${RED}[✗]${NC} $1"
}

log_step() {
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}▶ $1${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

# 打印启动横幅
clear
echo ""
echo "╔════════════════════════════════════════╗"
echo "║   ROS2  舵机控制系统     初始化           ║"
echo "╚════════════════════════════════════════╝"
echo ""
echo "  启动时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "  工作目录: $(pwd)"
echo ""

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 步骤 1: 加载 ROS 2 环境
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
log_step "步骤 1/4: 加载 ROS 2 环境"
log_info "ROS 发行版: ${ROS_DISTRO}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    log_success "ROS 2 环境加载完成"
else
    log_error "找不到 ROS 2 环境文件"
    exit 1
fi
echo ""

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 步骤 2: 构建工作空间
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
log_step "步骤 2/4: 构建工作空间"
log_info "开始编译，请稍候..."
echo ""

# 记录构建开始时间
BUILD_START=$(date +%s)

# 执行构建并显示简化输出
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 计算构建耗时
BUILD_END=$(date +%s)
BUILD_TIME=$((BUILD_END - BUILD_START))

echo ""
log_success "构建完成（耗时: ${BUILD_TIME}秒）"
echo ""

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 步骤 3: 加载工作空间环境
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
log_step "步骤 3/4: 加载工作空间环境"

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    log_success "工作空间环境加载完成"
else
    log_error "找不到 install/setup.bash"
    exit 1
fi
echo ""

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 步骤 4: 系统检查
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
log_step "步骤 4/4: 系统检查"

# 检查硬件设备
log_info "检查硬件设备..."
check_device() {
    local device=$1
    local name=$2
    if [ -e "$device" ]; then
        echo -e "  ${GREEN}✓${NC} $name: $device"
    else
        echo -e "  ${YELLOW}✗${NC} $name: $device ${YELLOW}(未找到)${NC}"
    fi
}

check_device "/dev/ttyUSB0" "USB串口"
check_device "/dev/ttyAMA0" "树莓派串口"
check_device "/dev/i2c-1" "I2C总线"

# 显示系统信息
echo ""
log_info "系统信息:"
echo "  • ROS 发行版: ${ROS_DISTRO}"
echo "  • ROS Domain ID: ${ROS_DOMAIN_ID:-0}"
echo "  • Python 版本: $(python3 --version 2>&1 | cut -d' ' -f2)"
echo "  • 工作空间: $(pwd)"

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 启动系统
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo ""
echo "╔════════════════════════════════════════╗"
echo "║        🚀 启动系统....                  ║"
echo "╚════════════════════════════════════════╝"
echo ""

# 延迟1秒让用户看到信息
sleep 1

# 选择启动模式
LAUNCH_MODE=${LAUNCH_MODE:-full}
LAUNCH_DEBUG=${LAUNCH_DEBUG:-false}
LAUNCH_BRIDGE_DEBUG=${LAUNCH_BRIDGE_DEBUG:-$LAUNCH_DEBUG}
LAUNCH_BUS_SERVO_DEBUG=${LAUNCH_BUS_SERVO_DEBUG:-true}
LAUNCH_IMU_DEBUG=${LAUNCH_IMU_DEBUG:-false}
LAUNCH_HEARTBEAT_DEBUG=${LAUNCH_HEARTBEAT_DEBUG:-false}
LAUNCH_PCA_DEBUG=${LAUNCH_PCA_DEBUG:-$LAUNCH_DEBUG}
LAUNCH_WS_DEBUG=${LAUNCH_WS_DEBUG:-false}
LAUNCH_DEBUG_AGGREGATE=${LAUNCH_DEBUG_AGGREGATE:-true}
LAUNCH_DEBUG_AGGREGATE_PERIOD=${LAUNCH_DEBUG_AGGREGATE_PERIOD:-1.0}
LAUNCH_DEBUG_AGGREGATE_MAX_LEN=${LAUNCH_DEBUG_AGGREGATE_MAX_LEN:-120}
LAUNCH_PROTOCOL_CACHE_FILE=${LAUNCH_PROTOCOL_CACHE_FILE:-/root/ros_ws/src/websocket/config/bus_protocol_cache.json}
LAUNCH_MANUAL_PROTOCOL_MAP_FILE=${LAUNCH_MANUAL_PROTOCOL_MAP_FILE:-}
LAUNCH_LX_ID_RANGES=${LAUNCH_LX_ID_RANGES:-21-34}
LAUNCH_ZL_ID_RANGES=${LAUNCH_ZL_ID_RANGES:-35-43}
LAUNCH_PROBE_ON_STARTUP=${LAUNCH_PROBE_ON_STARTUP:-true}
LAUNCH_PROBE_TIMEOUT_SEC=${LAUNCH_PROBE_TIMEOUT_SEC:-0.2}
LAUNCH_PROBE_ON_UNKNOWN_COMMAND=${LAUNCH_PROBE_ON_UNKNOWN_COMMAND:-true}
LAUNCH_PROBE_RETRY_INTERVAL_SEC=${LAUNCH_PROBE_RETRY_INTERVAL_SEC:-3.0}
LAUNCH_RUNTIME_PROBE_INTERVAL_SEC=${LAUNCH_RUNTIME_PROBE_INTERVAL_SEC:-0.05}
LAUNCH_READ_SERVICE_TIMEOUT_SEC=${LAUNCH_READ_SERVICE_TIMEOUT_SEC:-0.35}
PARALLEL_INSTANCES_FILE=${PARALLEL_INSTANCES_FILE:-/root/ros_ws/src/parallel_3dof_controller/config/parallel_3dof_instances.yaml}

if [ "$LAUNCH_MODE" = "multi" ]; then
    log_info "Launch mode: websocket_bus_servo + parallel_3dof_multi"
    launch_cmd=(
        ros2 launch websocket_bridge websocket_bus_servo.launch.py
        "instances_file:=$PARALLEL_INSTANCES_FILE"
        "debug:=$LAUNCH_DEBUG"
        "bridge_debug:=$LAUNCH_BRIDGE_DEBUG"
        "bus_servo_debug:=$LAUNCH_BUS_SERVO_DEBUG"
        "imu_debug:=$LAUNCH_IMU_DEBUG"
        "heartbeat_debug:=$LAUNCH_HEARTBEAT_DEBUG"
        "ws_debug:=$LAUNCH_WS_DEBUG"
        "debug_aggregate:=$LAUNCH_DEBUG_AGGREGATE"
        "debug_aggregate_period:=$LAUNCH_DEBUG_AGGREGATE_PERIOD"
        "debug_aggregate_max_len:=$LAUNCH_DEBUG_AGGREGATE_MAX_LEN"
        "protocol_cache_file:=$LAUNCH_PROTOCOL_CACHE_FILE"
        "lx_id_ranges:=$LAUNCH_LX_ID_RANGES"
        "zl_id_ranges:=$LAUNCH_ZL_ID_RANGES"
        "probe_on_startup:=$LAUNCH_PROBE_ON_STARTUP"
        "probe_timeout_sec:=$LAUNCH_PROBE_TIMEOUT_SEC"
        "probe_on_unknown_command:=$LAUNCH_PROBE_ON_UNKNOWN_COMMAND"
        "probe_retry_interval_sec:=$LAUNCH_PROBE_RETRY_INTERVAL_SEC"
        "runtime_probe_interval_sec:=$LAUNCH_RUNTIME_PROBE_INTERVAL_SEC"
        "read_service_timeout_sec:=$LAUNCH_READ_SERVICE_TIMEOUT_SEC"
    )
else
    log_info "Launch mode: full_system"
    launch_cmd=(
        ros2 launch websocket_bridge full_system.launch.py
        "debug:=$LAUNCH_DEBUG"
        "bridge_debug:=$LAUNCH_BRIDGE_DEBUG"
        "bus_servo_debug:=$LAUNCH_BUS_SERVO_DEBUG"
        "imu_debug:=$LAUNCH_IMU_DEBUG"
        "heartbeat_debug:=$LAUNCH_HEARTBEAT_DEBUG"
        "pca_debug:=$LAUNCH_PCA_DEBUG"
        "ws_debug:=$LAUNCH_WS_DEBUG"
        "debug_aggregate:=$LAUNCH_DEBUG_AGGREGATE"
        "debug_aggregate_period:=$LAUNCH_DEBUG_AGGREGATE_PERIOD"
        "debug_aggregate_max_len:=$LAUNCH_DEBUG_AGGREGATE_MAX_LEN"
        "protocol_cache_file:=$LAUNCH_PROTOCOL_CACHE_FILE"
        "lx_id_ranges:=$LAUNCH_LX_ID_RANGES"
        "zl_id_ranges:=$LAUNCH_ZL_ID_RANGES"
        "probe_on_startup:=$LAUNCH_PROBE_ON_STARTUP"
        "probe_timeout_sec:=$LAUNCH_PROBE_TIMEOUT_SEC"
        "probe_on_unknown_command:=$LAUNCH_PROBE_ON_UNKNOWN_COMMAND"
        "probe_retry_interval_sec:=$LAUNCH_PROBE_RETRY_INTERVAL_SEC"
        "runtime_probe_interval_sec:=$LAUNCH_RUNTIME_PROBE_INTERVAL_SEC"
        "read_service_timeout_sec:=$LAUNCH_READ_SERVICE_TIMEOUT_SEC"
    )
fi

if [ -n "$LAUNCH_MANUAL_PROTOCOL_MAP_FILE" ]; then
    launch_cmd+=("manual_protocol_map_file:=$LAUNCH_MANUAL_PROTOCOL_MAP_FILE")
fi

exec "${launch_cmd[@]}"
