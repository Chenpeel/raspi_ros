"""
完整系统Launch文件 - WebSocket + 舵机驱动

启动组件:
1. WebSocket桥接节点 (bridge_node)
2. 总线舵机驱动 (bus_servo_driver) - 从 bus_servo_map.json 动态加载
3. PCA9685舵机驱动 (pca_servo_driver)

数据流:
WebSocket客户端 <--> bridge_node <--> /servo/command & /servo/state <--> 舵机驱动
"""

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成Launch描述"""

    # 读取舵机映射配置
    pkg_share = get_package_share_directory('websocket_bridge')
    config_file = os.path.join(pkg_share, 'config', 'bus_servo_map.json')

    # 如果配置文件不存在，尝试从源码路径读取
    if not os.path.exists(config_file):
        # 开发环境下的路径
        config_file = os.path.join(
            Path(__file__).parent.parent, 'config', 'bus_servo_map.json'
        )

    # 读取配置
    servo_map = {}
    if os.path.exists(config_file):
        with open(config_file, 'r', encoding='utf-8') as f:
            servo_map = json.load(f)
        print(f"✓ 已加载舵机映射配置: {config_file}")
        print(f"  配置内容: {servo_map}")
    else:
        print(f"⚠ 警告: 舵机映射配置文件不存在: {config_file}")
        print(f"  将使用默认配置")
        # 默认配置（向后兼容）
        servo_map = {
            "/dev/ttyAMA0": [1, 2],
            "/dev/ttyAMA1": [7, 3, 9, 10, 11],
            "/dev/ttyAMA2": [4, 5],
            "/dev/ttyAMA3": [8, 6, 12, 13, 14]
        }

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

    # IMU 传感器参数
    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyAMA4',
        description='IMU 串口设备路径'
    )

    imu_baudrate_arg = DeclareLaunchArgument(
        'imu_baudrate',
        default_value='115200',
        description='IMU 串口波特率'
    )

    imu_debug_arg = DeclareLaunchArgument(
        'imu_debug',
        default_value=LaunchConfiguration('debug'),
        description='IMU 是否启用调试模式'
    )

    imu_publish_rate_arg = DeclareLaunchArgument(
        'imu_publish_rate',
        default_value='50.0',
        description='IMU 数据发布频率(Hz)'
    )

    imu_algo_type_arg = DeclareLaunchArgument(
        'imu_algo_type',
        default_value='9',
        description='IMU 融合算法类型(6=六轴, 9=九轴)'
    )

    imu_sensor_id_arg = DeclareLaunchArgument(
        'imu_sensor_id',
        default_value='0',
        description='IMU 传感器ID'
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
    imu_port = LaunchConfiguration('imu_port')
    imu_baudrate = LaunchConfiguration('imu_baudrate')
    imu_debug = LaunchConfiguration('imu_debug')
    imu_publish_rate = LaunchConfiguration('imu_publish_rate')
    imu_algo_type = LaunchConfiguration('imu_algo_type')
    imu_sensor_id = LaunchConfiguration('imu_sensor_id')

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

    # 2. 动态创建总线舵机驱动节点（从 bus_servo_map.json 读取）
    bus_servo_nodes = []
    servo_info_lines = []

    for idx, (port, servo_ids) in enumerate(servo_map.items()):
        # 跳过空列表端口（避免 ROS 2 参数类型推断错误）
        if not servo_ids:
            # 记录跳过信息到启动日志
            servo_info_lines.append(f'    - {port} @ 115200 bps (舵机ID: 无 - 已跳过)\n')
            print(f'  ⊘ 跳过端口 {port}（未配置舵机ID）')
            continue

        # 创建节点
        node = Node(
            package='servo_hardware',
            executable='bus_servo_driver',
            name=f'bus_servo_driver_{idx}',
            output='screen',
            parameters=[
                {'port': port},
                {'baudrate': baudrate},
                {'default_speed': 100},
                {'servo_ids': servo_ids},
                {'debug': debug},
                {'log_id': True}
            ],
            remappings=[
                ('~/command', '/servo/command'),
                ('~/state', '/servo/state'),
            ]
        )
        bus_servo_nodes.append(node)

        # 生成启动信息行
        servo_ids_str = ', '.join(map(str, servo_ids))
        servo_info_lines.append(f'    - {port} @ 115200 bps (舵机ID: {servo_ids_str})\n')

    # 3. IMU 传感器串口驱动节点（使用串口而非I2C）
    imu_driver_node = Node(
        package='servo_hardware',
        executable='imu_serial_driver',  # 使用串口版本
        name='imu_serial_driver',
        output='screen',
        parameters=[
            {'port': imu_port},  # 串口设备
            {'baudrate': imu_baudrate},  # 串口波特率
            {'publish_rate': imu_publish_rate},
            {'debug': imu_debug},
            {'algo_type': imu_algo_type},
            {'calibrate_on_start': False},
            {'sensor_id': imu_sensor_id}
        ],
        remappings=[
            ('~/data', '/sensor/imu'),
        ]
    )

    # 6. PCA9685舵机驱动节点 (已临时禁用 - I2C设备未连接导致树莓派关机)
    # pca_servo_node = Node(
    #     package='servo_hardware',
    #     executable='pca_servo_driver',
    #     name='pca_servo_driver',
    #     output='screen',
    #     parameters=[
    #         {'i2c_address': i2c_address},
    #         {'bus_number': i2c_bus},
    #         {'frequency': 50},
    #         {'min_pwm': 110},
    #         {'max_pwm': 520},
    #         {'min_us': 500},
    #         {'max_us': 2500},
    #         {'debug': debug}
    #     ],
    #     remappings=[
    #         ('~/command', '/servo/command'),
    #         ('~/state', '/servo/state'),
    #     ]
    # )

    # 启动信息
    log_info = LogInfo(
        msg=[
            '========================================\n',
            '  完整舵机控制系统已启动\n',
            '========================================\n',
            '  WebSocket: ws://', ws_host, ':', ws_port, '\n',
            '  设备ID: ', device_id, '\n',
            '  总线舵机驱动板:\n',
            *servo_info_lines,  # 动态生成的舵机信息
            '  IMU传感器: ', imu_port, ' @ ', imu_baudrate, ' bps, 频率=', imu_publish_rate, 'Hz\n',
            '  PCA9685: 已禁用 (I2C设备未连接)\n',
            '  调试模式: ', debug, '\n',
            '========================================\n',
            '  ROS 2话题:\n',
            '    发布: /servo/command (舵机控制命令)\n',
            '    订阅: /servo/state (舵机状态反馈)\n',
            '    订阅: /sensor/imu (IMU传感器数据)\n',
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
        imu_port_arg,  # IMU 串口设备
        imu_baudrate_arg,  # IMU 串口波特率
        imu_debug_arg,
        imu_publish_rate_arg,
        imu_algo_type_arg,
        imu_sensor_id_arg,

        # 启动信息
        log_info,

        # 节点
        bridge_node,
        *bus_servo_nodes,  # 动态生成的所有总线舵机驱动节点
        imu_driver_node,   # IMU 传感器串口驱动节点
        # pca_servo_node,  # 已临时禁用 - I2C设备未连接
    ])
