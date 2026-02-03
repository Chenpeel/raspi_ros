"""
简化版Launch文件 - WebSocket + 总线舵机

仅启动:
1. WebSocket桥接节点
2. 总线舵机驱动
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成Launch描述"""

    # 参数声明
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='启用调试模式'
    )

    bridge_debug_arg = DeclareLaunchArgument(
        'bridge_debug',
        default_value=LaunchConfiguration('debug'),
        description='WebSocket桥接节点调试模式'
    )

    bus_servo_debug_arg = DeclareLaunchArgument(
        'bus_servo_debug',
        default_value='true',
        description='总线舵机驱动调试模式'
    )

    heartbeat_debug_arg = DeclareLaunchArgument(
        'heartbeat_debug',
        default_value='false',
        description='心跳调试模式'
    )

    ws_debug_arg = DeclareLaunchArgument(
        'ws_debug',
        default_value='false',
        description='WebSocket服务端调试模式'
    )

    serial_port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyAMA0',
        description='串口设备路径'
    )

    instances_file_arg = DeclareLaunchArgument(
        'instances_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('parallel_3dof_controller'),
            'config',
            'parallel_3dof_instances.yaml'
        ]),
        description='parallel_3dof_controller multi-instance config file'
    )

    imu_debug_arg = DeclareLaunchArgument(
        'imu_debug',
        default_value='false',
        description='IMU 是否启用调试模式'
    )

    # WebSocket桥接节点
    bridge_node = Node(
        package='websocket_bridge',
        executable='bridge_node',
        name='websocket_ros2_bridge',
        output='screen',
        parameters=[
            {'debug': LaunchConfiguration('bridge_debug')},
            {'imu_debug': LaunchConfiguration('imu_debug')},
            {'heartbeat_debug': LaunchConfiguration('heartbeat_debug')},
            {'ws_debug': LaunchConfiguration('ws_debug')}
        ]
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
            {'debug': LaunchConfiguration('bus_servo_debug')},
            {'log_id': True}
        ]
    )

    # 3-DOF并联控制器多实例
    parallel_3dof_multi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('parallel_3dof_controller'),
            'launch',
            'parallel_3dof_multi.launch.py'
        ])),
        launch_arguments={
            'instances_file': LaunchConfiguration('instances_file')
        }.items()
    )

    return LaunchDescription([
        debug_arg,
        bridge_debug_arg,
        bus_servo_debug_arg,
        serial_port_arg,
        instances_file_arg,
        heartbeat_debug_arg,
        ws_debug_arg,
        imu_debug_arg,
        bridge_node,
        bus_servo_node,
        parallel_3dof_multi,
    ])
