"""
简化版Launch文件 - WebSocket + 总线舵机

仅启动:
1. WebSocket桥接节点
2. 总线舵机驱动
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成Launch描述"""

    # 参数声明
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='启用调试模式'
    )

    serial_port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyAMA0',
        description='串口设备路径'
    )

    # WebSocket桥接节点
    bridge_node = Node(
        package='websocket_bridge',
        executable='bridge_node',
        name='websocket_ros2_bridge',
        output='screen'
    )

    # 总线舵机驱动
    bus_servo_node = Node(
        package='servo_hardware',
        executable='bus_servo_driver',
        name='bus_servo_driver',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baudrate': 115200},
            {'debug': LaunchConfiguration('debug')},
            {'log_id': True}
        ]
    )

    return LaunchDescription([
        debug_arg,
        serial_port_arg,
        bridge_node,
        bus_servo_node,
    ])
