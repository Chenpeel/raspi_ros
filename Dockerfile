# ROS 2舵机控制系统Docker配置 - 支持多架构

# 声明 BuildKit 自动注入的平台变量
ARG TARGETPLATFORM
ARG BUILDPLATFORM
ARG TARGETOS
ARG TARGETARCH

# 使用自动平台检测（不指定 --platform，让 Docker 自动匹配）
FROM ros:jazzy-ros-base

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 配置 bash 为默认 shell
SHELL ["/bin/bash", "-c"]

# 架构检测并设置环境变量
RUN set -eux; \
    ARCH=$(uname -m); \
    echo "检测到架构: $ARCH"; \
    case "$ARCH" in \
        aarch64|armv8*) \
            echo "ARCH=arm64" > /etc/arch_env; \
            echo "设置架构: arm64" ;; \
        armv7*) \
            echo "ARCH=arm32" > /etc/arch_env; \
            echo "设置架构: arm32" ;; \
        x86_64|amd64) \
            echo "ARCH=amd64" > /etc/arch_env; \
            echo "设置架构: amd64" ;; \
        *) \
            echo "ARCH=unknown" > /etc/arch_env; \
            echo "警告: 未知架构 $ARCH" ;; \
    esac; \
    cat /etc/arch_env

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
