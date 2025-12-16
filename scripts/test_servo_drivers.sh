#!/bin/bash
# 测试servo_hardware包中的两个驱动节点

set -e

echo "================================================================="
echo "servo_hardware驱动测试脚本"
echo "================================================================="

# 加载ROS2环境
source install/setup.zsh

echo ""
echo "步骤1: 验证servo_msgs消息类型..."
echo "================================================================="

# 检查ServoCommand消息
if ros2 interface show servo_msgs/msg/ServoCommand >/dev/null 2>&1; then
    echo "✓ ServoCommand消息类型可用"
else
    echo "✗ ServoCommand消息类型不可用"
    exit 1
fi

# 检查ServoState消息
if ros2 interface show servo_msgs/msg/ServoState >/dev/null 2>&1; then
    echo "✓ ServoState消息类型可用"
else
    echo "✗ ServoState消息类型不可用"
    exit 1
fi

# 检查ServoStatus消息
if ros2 interface show servo_msgs/msg/ServoStatus >/dev/null 2>&1; then
    echo "✓ ServoStatus消息类型可用"
else
    echo "✗ ServoStatus消息类型不可用"
    exit 1
fi

echo ""
echo "步骤2: 检查驱动节点文件..."
echo "================================================================="

# 检查bus_servo.py
BUS_SERVO_FILE="src/hardware/servo_hardware/bus_servo.py"
if [ -f "$BUS_SERVO_FILE" ]; then
    if grep -q "from servo_msgs.msg import ServoCommand, ServoState" "$BUS_SERVO_FILE"; then
        echo "✓ bus_servo.py 已使用新消息类型"
    else
        echo "✗ bus_servo.py 未使用新消息类型"
        exit 1
    fi
else
    echo "✗ bus_servo.py 不存在"
    exit 1
fi

# 检查pca_servo.py
PCA_SERVO_FILE="src/hardware/servo_hardware/pca_servo.py"
if [ -f "$PCA_SERVO_FILE" ]; then
    if grep -q "from servo_msgs.msg import ServoCommand, ServoState" "$PCA_SERVO_FILE"; then
        echo "✓ pca_servo.py 已使用新消息类型"
    else
        echo "✗ pca_servo.py 未使用新消息类型"
        exit 1
    fi
else
    echo "✗ pca_servo.py 不存在"
    exit 1
fi

echo ""
echo "步骤3: 验证私有命名空间使用..."
echo "================================================================="

# 检查是否使用私有命名空间 ~/command
if grep -q "~/command" "$BUS_SERVO_FILE" && grep -q "~/command" "$PCA_SERVO_FILE"; then
    echo "✓ 驱动节点使用私有命名空间 ~/command"
else
    echo "✗ 驱动节点未使用私有命名空间"
    exit 1
fi

# 检查是否使用私有命名空间 ~/state
if grep -q "~/state" "$BUS_SERVO_FILE" && grep -q "~/state" "$PCA_SERVO_FILE"; then
    echo "✓ 驱动节点使用私有命名空间 ~/state"
else
    echo "✗ 驱动节点未使用私有命名空间"
    exit 1
fi

echo ""
echo "步骤4: 验证package.xml依赖..."
echo "================================================================="

PACKAGE_XML="src/hardware/package.xml"
if grep -q "<depend>servo_msgs</depend>" "$PACKAGE_XML"; then
    echo "✓ package.xml 包含servo_msgs依赖"
else
    echo "✗ package.xml 缺少servo_msgs依赖"
    exit 1
fi

echo ""
echo "步骤5: 验证编译..."
echo "================================================================="

# 编译servo_hardware包
if colcon build --packages-select servo_hardware 2>&1 | grep -q "Finished <<< servo_hardware"; then
    echo "✓ servo_hardware包编译成功"
else
    echo "✗ servo_hardware包编译失败"
    exit 1
fi

echo ""
echo "================================================================="
echo "✓ 所有测试通过！"
echo "================================================================="
echo ""
echo "摘要:"
echo "  - ServoCommand、ServoState、ServoStatus消息类型已定义"
echo "  - bus_servo.py 和 pca_servo.py 已重构使用新消息类型"
echo "  - 使用私有命名空间 ~/command 和 ~/state"
echo "  - package.xml 依赖已更新"
echo "  - 编译成功"
echo ""
echo "阶段2完成！"
echo "================================================================="

exit 0
