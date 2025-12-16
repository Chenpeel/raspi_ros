#!/bin/bash
# ROS2舵机控制系统依赖安装脚本

set -e  # 遇到错误立即退出

echo "========================================="
echo "  ROS2舵机控制系统依赖安装"
echo "========================================="

# 检查Python版本
PYTHON_VERSION=$(python --version | awk '{print $2}' | cut -d'.' -f1,2)
echo "检测到Python版本: $PYTHON_VERSION"

if [ "$PYTHON_VERSION" != "3.12" ]; then
    echo "❌ 错误: 需要Python 3.12，当前版本为 $PYTHON_VERSION"
    echo "请激活ros2 conda环境: conda activate ros2"
    exit 1
fi

# 安装Python依赖
echo ""
echo "正在安装Python依赖..."
pip install -r requirements.txt

echo ""
echo "✅ 所有依赖安装完成！"
echo ""
echo "下一步："
echo "  1. 清理旧构建: rm -rf build/ install/ log/"
echo "  2. 重新编译: colcon build"
echo "  3. source环境: source install/setup.zsh"
echo "========================================="
