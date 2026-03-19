# WebSocket 配置文件说明

本目录只保留 WebSocket 桥接自身使用的协议示例与流格式定义。

## 当前保留的配置文件

- `std_web2ros_stream.json` - Web → ROS 输入格式示例
- `std_ros2web_stream.json` - ROS → Web 输出格式示例

这些文件用于描述桥接层的消息结构，便于联调和协议对照。

## 已迁移的配置

硬件相关配置已迁移到 `servo_hardware` 包：

- 开发环境：`src/hardware/servo_hardware/config/bus_servo_map.json`
- 安装环境：`share/servo_hardware/config/bus_servo_map.json`
- 开发环境：`src/hardware/servo_hardware/config/manual_protocol_map.json`
- 安装环境：`share/servo_hardware/config/manual_protocol_map.json`

BVH 动作配置已迁移到 `record_load_action` 包：

- `src/record_load_action/README.md`
