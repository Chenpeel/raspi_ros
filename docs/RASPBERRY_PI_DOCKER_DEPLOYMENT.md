# 树莓派Docker部署指南

## 系统要求

- **硬件**: 树莓派 4B/5（推荐4GB+内存）
- **操作系统**: Raspberry Pi OS (64位) 或 Ubuntu 22.04/24.04 ARM64
- **Docker**: 20.10+
- **Docker Compose**: 2.0+

## 快速开始

### 1. 安装Docker（如果未安装）

```bash
# 安装Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# 添加当前用户到docker组（避免每次sudo）
sudo usermod -aG docker $USER

# 重新登录或执行
newgrp docker

# 安装Docker Compose
sudo apt-get update
sudo apt-get install -y docker-compose-plugin

# 验证安装
docker --version
docker compose version
```

### 2. 克隆项目（如果还没有）

```bash
# 在树莓派上克隆项目
cd ~
git clone <你的仓库URL> ros_project
cd ros_project
```

### 3. 启动开发环境

```bash
# 构建镜像并启动容器（首次运行需要几分钟）
docker compose build
docker compose up -d

# 进入容器
docker compose exec ros2_servo bash

# 在容器内：编译项目
cd /root/ros_ws
colcon build --packages-select websocket_bridge servo_hardware

# source环境
source install/setup.bash

# 启动系统
ros2 launch websocket_bridge full_system.launch.py
```

## 使用方式

### 开发模式（推荐）

**启动容器**：
```bash
# 启动并进入容器
docker compose up -d
docker compose exec ros2_servo bash
```

**在容器内开发**：
```bash
# 编译
colcon build

# 运行测试
ros2 run servo_hardware bus_servo_driver

# 启动完整系统
ros2 launch websocket_bridge full_system.launch.py
```

**退出容器**：
```bash
exit  # 容器会继续运行

# 停止容器
docker compose down
```

### 生产模式（自动启动）

```bash
# 启动生产模式（自动运行舵机控制系统）
docker compose --profile production up -d ros2_servo_prod

# 查看日志
docker compose logs -f ros2_servo_prod

# 停止
docker compose --profile production down
```

## 硬件设备配置

### 串口设备检查

```bash
# 在宿主机上检查串口设备
ls -l /dev/ttyUSB* /dev/ttyAMA*

# 应该看到类似：
# /dev/ttyUSB0 -> USB转串口
# /dev/ttyAMA0 -> 树莓派硬件串口
```

### I2C设备检查

```bash
# 启用I2C（如果未启用）
sudo raspi-config
# 选择: Interface Options → I2C → Enable

# 检查I2C设备
ls -l /dev/i2c-*

# 扫描I2C设备（需要i2c-tools）
sudo apt-get install -y i2c-tools
i2cdetect -y 1
```

### 权限配置

```bash
# 给串口设备添加权限
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyAMA0

# 永久配置：添加udev规则
sudo nano /etc/udev/rules.d/99-serial.rules

# 添加以下内容：
# KERNEL=="ttyUSB[0-9]*", MODE="0666"
# KERNEL=="ttyAMA[0-9]*", MODE="0666"

# 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Docker命令参考

### 容器管理

```bash
# 查看运行中的容器
docker compose ps

# 查看容器日志
docker compose logs -f ros2_servo

# 进入运行中的容器
docker compose exec ros2_servo bash

# 重启容器
docker compose restart ros2_servo

# 停止容器
docker compose down

# 停止并删除卷
docker compose down -v
```

### 镜像管理

```bash
# 重新构建镜像（代码更新后）
docker compose build --no-cache

# 查看镜像
docker images | grep ros

# 删除旧镜像
docker image prune -a
```

### 资源清理

```bash
# 清理未使用的容器
docker container prune

# 清理未使用的镜像
docker image prune -a

# 清理未使用的卷
docker volume prune

# 完整清理（谨慎使用）
docker system prune -a --volumes
```

## 文件结构

容器内的目录映射：

```
宿主机                      容器内
./src                 → /root/ros_ws/src
./build               → /root/ros_ws/build
./install             → /root/ros_ws/install
./log                 → /root/ros_ws/log
./.claude             → /root/ros_ws/.claude
./.TODOs              → /root/ros_ws/.TODOs
```

**注意**：
- `src/`：源代码，可以在宿主机编辑
- `build/`, `install/`, `log/`：编译输出，会自动同步

## 网络配置

### WebSocket端口

默认端口：`9102`

由于使用`network_mode: host`，容器直接使用宿主机网络：

```bash
# 在宿主机上验证端口
sudo netstat -tuln | grep 9102

# 从其他设备测试连接
wscat -c ws://树莓派IP:9102
```

### 防火墙配置

如果启用了防火墙，需要开放端口：

```bash
# 使用ufw
sudo ufw allow 9102/tcp

# 或使用iptables
sudo iptables -A INPUT -p tcp --dport 9102 -j ACCEPT
```

## 故障排查

### 问题1：容器无法访问串口

**症状**：
```
Permission denied: '/dev/ttyUSB0'
```

**解决**：
```bash
# 方法1：在宿主机上修改权限
sudo chmod 666 /dev/ttyUSB0

# 方法2：添加用户到dialout组（容器内）
docker compose exec ros2_servo usermod -a -G dialout root
docker compose restart ros2_servo
```

### 问题2：I2C设备找不到

**症状**：
```
No such file or directory: '/dev/i2c-1'
```

**解决**：
```bash
# 在树莓派上启用I2C
sudo raspi-config
# Interface Options → I2C → Enable

# 重启
sudo reboot
```

### 问题3：WebSocket无法连接

**症状**：
```
Connection refused
```

**检查**：
```bash
# 1. 检查容器是否运行
docker compose ps

# 2. 检查端口监听
docker compose exec ros2_servo netstat -tuln | grep 9102

# 3. 检查防火墙
sudo ufw status

# 4. 查看容器日志
docker compose logs -f ros2_servo
```

### 问题4：Python版本问题

ROS 2 Jazzy使用Python 3.12，镜像已包含正确版本。

如果遇到版本问题：
```bash
# 在容器内检查
python3 --version  # 应该是3.12.x
```

## 开发工作流

### 典型开发流程

```bash
# 1. 在宿主机编辑代码（使用VS Code等）
# 文件会自动同步到容器

# 2. 进入容器编译
docker compose exec ros2_servo bash
cd /root/ros_ws
colcon build --packages-select websocket_bridge servo_hardware

# 3. 测试
source install/setup.bash
ros2 run servo_hardware bus_servo_driver

# 4. 如果需要重启容器
exit
docker compose restart ros2_servo
```

### Git工作流

```bash
# 在容器内使用git
docker compose exec ros2_servo bash

git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

git status
git add .
git commit -m "描述"
git push
```

## 性能优化

### 减少镜像大小

当前Dockerfile已经优化：
- 使用`--no-cache-dir`减少pip缓存
- 使用`rm -rf /var/lib/apt/lists/*`清理apt缓存

### 加速编译

```bash
# 使用并行编译
colcon build --parallel-workers 2  # 树莓派4B推荐2
```

### 内存管理

树莓派内存有限，建议：
```bash
# 限制容器内存（修改docker-compose.yaml）
services:
  ros2_servo:
    mem_limit: 2g
    mem_reservation: 1g
```

## 生产部署

### 开机自启动

```bash
# 方法1：使用Docker restart策略
# 已在docker-compose.yaml中配置: restart: unless-stopped

# 方法2：systemd服务
sudo nano /etc/systemd/system/ros2-servo.service
```

systemd服务文件内容：
```ini
[Unit]
Description=ROS 2 Servo Control System
Requires=docker.service
After=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/pi/ros_project
ExecStart=/usr/bin/docker compose --profile production up -d
ExecStop=/usr/bin/docker compose --profile production down
Restart=always

[Install]
WantedBy=multi-user.target
```

启用服务：
```bash
sudo systemctl enable ros2-servo.service
sudo systemctl start ros2-servo.service
```

## 备份和恢复

### 备份配置

```bash
# 备份docker-compose.yaml和源代码
tar -czf ros2_backup_$(date +%Y%m%d).tar.gz \
  docker-compose.yaml Dockerfile src/ .claude/
```

### 恢复

```bash
# 解压备份
tar -xzf ros2_backup_20251215.tar.gz

# 重新构建和启动
docker compose build
docker compose up -d
```

## 监控和日志

### 查看日志

```bash
# 实时日志
docker compose logs -f ros2_servo

# 最近100行日志
docker compose logs --tail=100 ros2_servo

# 保存日志到文件
docker compose logs ros2_servo > servo_logs.txt
```

### 资源监控

```bash
# 查看容器资源使用
docker stats ros2_servo_control

# 查看系统资源
htop  # 需要安装: sudo apt install htop
```

---

**生成时间**: 2025-12-15
**维护者**: chenpeel
**Docker镜像**: ROS 2 Jazzy (ros:jazzy-ros-base)
**树莓派**: 4B/5 (ARM64)
