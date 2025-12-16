# ROS 2舵机控制系统Docker配置 - 支持多架构

# 使用TARGETPLATFORM构建参数自动检测平台架构
FROM --platform=${TARGETPLATFORM:-linux/amd64} ros:jazzy-ros-base

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 架构检测脚本 - 根据架构设置不同配置
RUN case $(uname -m) in \
        aarch64|armv8l) echo "ARCH=arm64" > /arch_env;; \
        armv7l) echo "ARCH=arm32" > /arch_env;; \
        x86_64) echo "ARCH=amd64" > /arch_env;; \
        *) echo "ARCH=unknown" > /arch_env;; \
    esac

# 源文件架构检测
SHELL ["/bin/bash", "-c"]
RUN cat /arch_env && source /arch_env

# 更新apt源并安装基础工具
RUN apt-get update && apt-get install -y \
    git \
    vim \
    nano \
    wget \
    curl \
    build-essential \
    cmake \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# 安装Python依赖
RUN pip3 install --break-system-packages --no-cache-dir \
    websockets \
    pyserial \
    smbus2

# 安装ROS 2依赖
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
RUN mkdir -p /root/ros_ws/src

# 设置工作目录
WORKDIR /root/ros_ws

# 配置ROS环境
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /root/ros_ws/install/setup.bash ]; then source /root/ros_ws/install/setup.bash; fi" >> /root/.bashrc

# 配置串口和I2C权限
RUN usermod -a -G dialout root || true
RUN usermod -a -G i2c root || true

# 暴露WebSocket端口
EXPOSE 9102

# 默认命令
CMD ["/bin/bash"]
