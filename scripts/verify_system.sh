#!/bin/bash
# ROS2舵机系统验证脚本 - 每个重构阶段完成后运行

set -e  # 遇到错误立即退出

echo "========================================="
echo "  ROS2舵机系统验证脚本"
echo "  时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "========================================="
echo ""

# 切换到workspace根目录
cd "$(dirname "$0")/.."
WORKSPACE_ROOT=$(pwd)

# 颜色输出
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 成功计数
PASS_COUNT=0
FAIL_COUNT=0

# 打印测试结果
print_result() {
    local test_name=$1
    local result=$2
    if [ "$result" -eq 0 ]; then
        echo -e "[${GREEN}✓${NC}] $test_name"
        ((PASS_COUNT++))
    else
        echo -e "[${RED}✗${NC}] $test_name"
        ((FAIL_COUNT++))
    fi
}

# ============================================
# 1. 编译检查
# ============================================
echo "[1/6] 编译检查..."
START_TIME=$(date +%s)

# 清理旧的build
if colcon build --symlink-install 2>&1 | tee /tmp/build.log; then
    print_result "编译检查" 0
else
    print_result "编译检查" 1
    echo -e "${YELLOW}编译日志:${NC}"
    tail -n 20 /tmp/build.log
fi

END_TIME=$(date +%s)
BUILD_TIME=$((END_TIME - START_TIME))
echo "  编译耗时: ${BUILD_TIME}秒"
echo ""

# ============================================
# 2. 包依赖检查
# ============================================
echo "[2/6] 包依赖检查..."

# Source环境
source install/setup.bash

# 检查所有包
if rosdep check --from-paths src --ignore-src -y >/dev/null 2>&1; then
    print_result "包依赖检查" 0
else
    print_result "包依赖检查" 1
    echo -e "${YELLOW}缺失的依赖:${NC}"
    rosdep check --from-paths src --ignore-src -y 2>&1 | grep "ERROR"
fi
echo ""

# ============================================
# 3. 包列表检查
# ============================================
echo "[3/6] ROS2包列表检查..."

# 预期的包列表（根据重构阶段动态调整）
EXPECTED_PACKAGES=(
    "servo_hardware"
    "websocket_bridge"
)

# 如果servo_msgs存在，检查它
if [ -d "src/servo_msgs" ]; then
    EXPECTED_PACKAGES+=("servo_msgs")
fi

# 如果servo_hardware_interface存在，检查它
if [ -d "src/servo_hardware_interface" ]; then
    EXPECTED_PACKAGES+=("servo_hardware_interface")
fi

for pkg in "${EXPECTED_PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "^${pkg}$"; then
        print_result "包存在: $pkg" 0
    else
        print_result "包存在: $pkg" 1
    fi
done
echo ""

# ============================================
# 4. 消息类型检查（如果servo_msgs已创建）
# ============================================
if [ -d "src/servo_msgs" ]; then
    echo "[4/6] 消息类型检查..."

    EXPECTED_MSGS=(
        "servo_msgs/msg/ServoCommand"
        "servo_msgs/msg/ServoState"
        "servo_msgs/msg/ServoStatus"
    )

    for msg in "${EXPECTED_MSGS[@]}"; do
        if ros2 interface show "$msg" >/dev/null 2>&1; then
            print_result "消息类型: $msg" 0
        else
            print_result "消息类型: $msg" 1
        fi
    done
    echo ""
else
    echo "[4/6] 消息类型检查 - 跳过（servo_msgs未创建）"
    echo ""
fi

# ============================================
# 5. 节点可执行性检查
# ============================================
echo "[5/6] 节点可执行性检查..."

# 检查驱动节点
if ros2 pkg executables servo_hardware | grep -q "bus_servo_driver"; then
    print_result "节点可执行: bus_servo_driver" 0
else
    print_result "节点可执行: bus_servo_driver" 1
fi

if ros2 pkg executables servo_hardware | grep -q "pca_servo_driver"; then
    print_result "节点可执行: pca_servo_driver" 0
else
    print_result "节点可执行: pca_servo_driver" 1
fi

# 检查桥接节点
if ros2 pkg executables websocket_bridge | grep -q "bridge_node"; then
    print_result "节点可执行: bridge_node" 0
else
    print_result "节点可执行: bridge_node" 1
fi
echo ""

# ============================================
# 6. Launch文件检查
# ============================================
echo "[6/6] Launch文件检查..."

LAUNCH_FILES=(
    "src/websocket/launch/websocket_bus_servo.launch.py"
    "src/websocket/launch/full_system.launch.py"
)

for launch_file in "${LAUNCH_FILES[@]}"; do
    if [ -f "$launch_file" ]; then
        # 简单语法检查
        if python3 -m py_compile "$launch_file" 2>/dev/null; then
            print_result "Launch文件语法: $(basename $launch_file)" 0
        else
            print_result "Launch文件语法: $(basename $launch_file)" 1
        fi
    else
        print_result "Launch文件存在: $(basename $launch_file)" 1
    fi
done
echo ""

# ============================================
# 汇总结果
# ============================================
echo "========================================="
echo "  验证结果汇总"
echo "========================================="
TOTAL_TESTS=$((PASS_COUNT + FAIL_COUNT))
echo -e "总测试数: $TOTAL_TESTS"
echo -e "${GREEN}通过: $PASS_COUNT${NC}"
echo -e "${RED}失败: $FAIL_COUNT${NC}"
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "${GREEN}✓ 所有验证通过！系统处于健康状态。${NC}"
    exit 0
else
    echo -e "${RED}✗ 存在 $FAIL_COUNT 个失败项，请检查并修复。${NC}"
    exit 1
fi
