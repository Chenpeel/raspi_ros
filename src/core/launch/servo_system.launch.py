"""Launch file for Servo Control system"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # 获取配置文件路径
    config_dir = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config'
    )
    params_file = os.path.join(config_dir, 'servo_params.yaml')

    # 声明参数
    ws_host_arg = DeclareLaunchArgument(
        'ws_host',
        default_value='0.0.0.0',
        description='WebSocket server host'
    )

    ws_port_arg = DeclareLaunchArgument(
        'ws_port',
        default_value='9102',
        description='WebSocket server port'
    )

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='default',
        description='Device ID'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )

    # 舵机控制节点
    servo_control_node = Node(
        package='servo_control',
        executable='servo_control_node',
        name='servo_control',
        output='screen',
        parameters=[
            {
                'ws_host': LaunchConfiguration('ws_host'),
                'ws_port': LaunchConfiguration('ws_port'),
                'device_id': LaunchConfiguration('device_id'),
                'debug': LaunchConfiguration('debug'),
            }
        ]
    )

    return LaunchDescription([
        ws_host_arg,
        ws_port_arg,
        device_id_arg,
        debug_arg,
        servo_control_node,
    ])
