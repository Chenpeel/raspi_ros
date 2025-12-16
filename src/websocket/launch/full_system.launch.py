"""
完整系统Launch文件 - WebSocket + 舵机驱动

启动组件:
1. WebSocket桥接节点 (bridge_node)
2. 总线舵机驱动 (bus_servo_driver)
3. PCA9685舵机驱动 (pca_servo_driver)

数据流:
WebSocket客户端 <--> bridge_node <--> /servo/command & /servo/state <--> 舵机驱动
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成Launch描述"""

    # 声明Launch参数
    ws_host_arg = DeclareLaunchArgument(
        'ws_host',
        default_value='0.0.0.0',
        description='WebSocket服务器监听地址'
    )

    ws_port_arg = DeclareLaunchArgument(
        'ws_port',
        default_value='9105',
        description='WebSocket服务器监听端口'
    )

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='robot',
        description='设备ID'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='是否启用调试模式'
    )

    # 串口设备参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyAMA0',
        description='总线舵机串口设备'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='总线舵机波特率'
    )

    # I2C参数
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='64',  # 0x40 = 64
        description='PCA9685 I2C地址 (十进制)'
    )

    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='1',
        description='I2C总线号'
    )

    # 获取参数值
    ws_host = LaunchConfiguration('ws_host')
    ws_port = LaunchConfiguration('ws_port')
    device_id = LaunchConfiguration('device_id')
    debug = LaunchConfiguration('debug')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    i2c_address = LaunchConfiguration('i2c_address')
    i2c_bus = LaunchConfiguration('i2c_bus')

    # 1. WebSocket桥接节点
    bridge_node = Node(
        package='websocket_bridge',
        executable='bridge_node',
        name='websocket_ros2_bridge',
        output='screen',
        parameters=[
            {'ws_host': ws_host},
            {'ws_port': ws_port},
            {'device_id': device_id},
            {'debug': debug}
        ],
        remappings=[
            # 如果需要重新映射话题名称，可以在这里配置
        ]
    )

    # 2. 总线舵机驱动 - 串口0 (ttyAMA0, 舵机ID: 1, 2)
    bus_servo_node_0 = Node(
        package='servo_hardware',
        executable='bus_servo_driver',
        name='bus_servo_driver_0',
        output='screen',
        parameters=[
            {'port': '/dev/ttyAMA0'},
            {'baudrate': baudrate},
            {'default_speed': 100},
            {'servo_ids': [1, 2]},
            {'debug': debug},
            {'log_id': True}
        ],
        remappings=[
            ('~/command', '/servo/command'),
            ('~/state', '/servo/state'),
        ]
    )

    # 3. 总线舵机驱动 - 串口1 (ttyAMA1, 舵机ID: 7, 3, 9, 10, 11)
    bus_servo_node_1 = Node(
        package='servo_hardware',
        executable='bus_servo_driver',
        name='bus_servo_driver_1',
        output='screen',
        parameters=[
            {'port': '/dev/ttyAMA1'},
            {'baudrate': baudrate},
            {'default_speed': 100},
            {'servo_ids': [7, 3, 9, 10, 11]},
            {'debug': debug},
            {'log_id': True}
        ],
        remappings=[
            ('~/command', '/servo/command'),
            ('~/state', '/servo/state'),
        ]
    )

    # 4. 总线舵机驱动 - 串口2 (ttyAMA2, 舵机ID: 4, 5)
    bus_servo_node_2 = Node(
        package='servo_hardware',
        executable='bus_servo_driver',
        name='bus_servo_driver_2',
        output='screen',
        parameters=[
            {'port': '/dev/ttyAMA2'},
            {'baudrate': baudrate},
            {'default_speed': 100},
            {'servo_ids': [4, 5]},
            {'debug': debug},
            {'log_id': True}
        ],
        remappings=[
            ('~/command', '/servo/command'),
            ('~/state', '/servo/state'),
        ]
    )

    # 5. 总线舵机驱动 - 串口3 (ttyAMA3, 舵机ID: 8, 6, 12, 13, 14)
    bus_servo_node_3 = Node(
        package='servo_hardware',
        executable='bus_servo_driver',
        name='bus_servo_driver_3',
        output='screen',
        parameters=[
            {'port': '/dev/ttyAMA3'},
            {'baudrate': baudrate},
            {'default_speed': 100},
            {'servo_ids': [8, 6, 12, 13, 14]},
            {'debug': debug},
            {'log_id': True}
        ],
        remappings=[
            ('~/command', '/servo/command'),
            ('~/state', '/servo/state'),
        ]
    )

    # 6. PCA9685舵机驱动节点
    pca_servo_node = Node(
        package='servo_hardware',
        executable='pca_servo_driver',
        name='pca_servo_driver',
        output='screen',
        parameters=[
            {'i2c_address': i2c_address},
            {'bus_number': i2c_bus},
            {'frequency': 50},
            {'min_pwm': 110},
            {'max_pwm': 520},
            {'min_us': 500},
            {'max_us': 2500},
            {'debug': debug}
        ],
        remappings=[
            ('~/command', '/servo/command'),
            ('~/state', '/servo/state'),
        ]
    )

    # 启动信息
    log_info = LogInfo(
        msg=[
            '========================================\n',
            '  完整舵机控制系统已启动\n',
            '========================================\n',
            '  WebSocket: ws://', ws_host, ':', ws_port, '\n',
            '  设备ID: ', device_id, '\n',
            '  总线舵机驱动板:\n',
            '    - /dev/ttyAMA0 @ ', baudrate, ' bps (舵机ID: 1, 2)\n',
            '    - /dev/ttyAMA1 @ ', baudrate, ' bps (舵机ID: 3, 7, 9, 10, 11)\n',
            '    - /dev/ttyAMA2 @ ', baudrate, ' bps (舵机ID: 4, 5)\n',
            '    - /dev/ttyAMA3 @ ', baudrate, ' bps (舵机ID: 6, 8, 12, 13, 14)\n',
            '  PCA9685: I2C地址=0x', i2c_address, ' 总线=', i2c_bus, '\n',
            '  调试模式: ', debug, '\n',
            '========================================\n',
            '  ROS 2话题:\n',
            '    发布: /servo/command (舵机控制命令)\n',
            '    订阅: /servo/state (舵机状态反馈)\n',
            '========================================\n'
        ]
    )

    return LaunchDescription([
        # 参数声明
        ws_host_arg,
        ws_port_arg,
        device_id_arg,
        debug_arg,
        serial_port_arg,
        baudrate_arg,
        i2c_address_arg,
        i2c_bus_arg,

        # 启动信息
        log_info,

        # 节点
        bridge_node,
        bus_servo_node_0,
        bus_servo_node_1,
        bus_servo_node_2,
        bus_servo_node_3,
        pca_servo_node,
    ])
