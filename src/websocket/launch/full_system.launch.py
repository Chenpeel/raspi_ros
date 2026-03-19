"""
完整系统Launch文件 - WebSocket + 舵机驱动

启动组件:
1. WebSocket桥接节点 (bridge_node)
2. 总线舵机串口驱动 (bus_port_driver) - 从 bus_servo_map.json 动态加载
3. 总线协议路由 (bus_protocol_router) - 协议识别/缓存/路由
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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成Launch描述"""

    # 读取硬件配置（优先源码路径，确保与开发现场配置一致）
    hardware_pkg_share = get_package_share_directory('servo_hardware')
    workspace_source_config_file = (
        '/root/ros_ws/src/hardware/servo_hardware/config/bus_servo_map.json'
    )
    cwd_source_config_file = os.path.join(
        Path.cwd(), 'src', 'hardware', 'servo_hardware', 'config', 'bus_servo_map.json'
    )
    source_config_file = os.path.join(
        Path(__file__).resolve().parent.parent.parent,
        'hardware',
        'servo_hardware',
        'config',
        'bus_servo_map.json',
    )
    installed_config_file = os.path.join(hardware_pkg_share, 'config', 'bus_servo_map.json')
    config_candidates = [
        workspace_source_config_file,
        cwd_source_config_file,
        source_config_file,
        installed_config_file,
    ]
    config_file = next(
        (path for path in config_candidates if os.path.exists(path)),
        source_config_file,
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

    # 协议缓存默认路径（与舵机映射配置同目录，便于持久化）
    protocol_cache_default = os.path.join(
        os.path.dirname(config_file),
        'bus_protocol_cache.json'
    )

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

    protocol_cache_file_arg = DeclareLaunchArgument(
        'protocol_cache_file',
        default_value=protocol_cache_default,
        description='总线协议探测缓存文件路径'
    )

    manual_protocol_map_file_arg = DeclareLaunchArgument(
        'manual_protocol_map_file',
        default_value='',
        description='手动协议映射文件(可选，优先级高于缓存/范围)'
    )

    lx_id_ranges_arg = DeclareLaunchArgument(
        'lx_id_ranges',
        default_value='21-34',
        description='幻尔协议ID范围(逗号分隔，如21-34,60-64)'
    )

    zl_id_ranges_arg = DeclareLaunchArgument(
        'zl_id_ranges',
        default_value='35-43',
        description='众灵协议ID范围(逗号分隔，如35-43)'
    )

    probe_on_startup_arg = DeclareLaunchArgument(
        'probe_on_startup',
        default_value='true',
        description='是否在启动时探测未知ID协议'
    )

    probe_timeout_sec_arg = DeclareLaunchArgument(
        'probe_timeout_sec',
        default_value='0.2',
        description='单次协议探测超时(秒)'
    )

    probe_on_unknown_command_arg = DeclareLaunchArgument(
        'probe_on_unknown_command',
        default_value='true',
        description='未知ID收到命令时是否触发在线探测'
    )

    probe_retry_interval_sec_arg = DeclareLaunchArgument(
        'probe_retry_interval_sec',
        default_value='3.0',
        description='未知ID在线探测失败后重试最小间隔(秒)'
    )

    runtime_probe_interval_sec_arg = DeclareLaunchArgument(
        'runtime_probe_interval_sec',
        default_value='0.05',
        description='后台在线探测任务轮询周期(秒)'
    )

    read_service_timeout_sec_arg = DeclareLaunchArgument(
        'read_service_timeout_sec',
        default_value='0.35',
        description='全局读角度服务超时(秒)'
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

    debug_aggregate_arg = DeclareLaunchArgument(
        'debug_aggregate',
        default_value='true',
        description='是否聚合调试日志'
    )

    debug_aggregate_period_arg = DeclareLaunchArgument(
        'debug_aggregate_period',
        default_value='1.0',
        description='调试日志聚合刷新周期(秒)'
    )

    debug_aggregate_max_len_arg = DeclareLaunchArgument(
        'debug_aggregate_max_len',
        default_value='120',
        description='调试日志聚合单条最大长度'
    )

    bvh_action_file_arg = DeclareLaunchArgument(
        'bvh_action_file',
        default_value='',
        description='BVH动作配置文件路径'
    )

    isaac_bridge_enable_arg = DeclareLaunchArgument(
        'enable_isaac_bridge',
        default_value='true',
        description='是否启动Isaac-ROS桥接节点'
    )

    isaac_bridge_debug_arg = DeclareLaunchArgument(
        'isaac_bridge_debug',
        default_value='false',
        description='Isaac-ROS桥接节点调试模式'
    )

    isaac_command_topic_arg = DeclareLaunchArgument(
        'isaac_command_topic',
        default_value='/sim/servo_command',
        description='Isaac侧舵机命令话题'
    )

    isaac_state_topic_arg = DeclareLaunchArgument(
        'isaac_state_topic',
        default_value='/sim/servo_state',
        description='Isaac侧舵机状态话题'
    )

    isaac_enforce_limits_arg = DeclareLaunchArgument(
        'isaac_enforce_limits',
        default_value='true',
        description='Isaac桥接是否强制位置限幅'
    )

    enable_sim_cpp_bridge_arg = DeclareLaunchArgument(
        'enable_sim_cpp_bridge',
        default_value='false',
        description='是否启动C++仿真桥接节点(sim_servo_bridge_cpp)'
    )

    sim_cpp_bridge_debug_arg = DeclareLaunchArgument(
        'sim_cpp_bridge_debug',
        default_value='false',
        description='C++仿真桥接节点调试模式'
    )

    sim_joint_cmd_topic_arg = DeclareLaunchArgument(
        'sim_joint_cmd_topic',
        default_value='/sim/joint_cmd',
        description='仿真侧关节命令话题(std_msgs/Float32MultiArray)'
    )

    sim_joint_state_fb_topic_arg = DeclareLaunchArgument(
        'sim_joint_state_fb_topic',
        default_value='/sim/joint_state_fb',
        description='仿真侧关节反馈话题(std_msgs/Float32MultiArray)'
    )

    sim_publish_rate_hz_arg = DeclareLaunchArgument(
        'sim_publish_rate_hz',
        default_value='50.0',
        description='C++仿真桥接发布频率(Hz)'
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
        default_value='false',
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
    imu_enable_arg = DeclareLaunchArgument(
        'imu_enable',
        default_value='false',
        description='是否启动IMU节点'
    )

    # 获取参数值
    ws_host = LaunchConfiguration('ws_host')
    ws_port = LaunchConfiguration('ws_port')
    device_id = LaunchConfiguration('device_id')
    debug = LaunchConfiguration('debug')
    baudrate = LaunchConfiguration('baudrate')
    i2c_address = LaunchConfiguration('i2c_address')
    i2c_bus = LaunchConfiguration('i2c_bus')
    imu_port = LaunchConfiguration('imu_port')
    imu_baudrate = LaunchConfiguration('imu_baudrate')
    imu_debug = LaunchConfiguration('imu_debug')
    imu_publish_rate = LaunchConfiguration('imu_publish_rate')
    imu_algo_type = LaunchConfiguration('imu_algo_type')
    imu_sensor_id = LaunchConfiguration('imu_sensor_id')
    imu_enable = LaunchConfiguration('imu_enable')
    bridge_debug = LaunchConfiguration('bridge_debug')
    bus_servo_debug = LaunchConfiguration('bus_servo_debug')
    protocol_cache_file = LaunchConfiguration('protocol_cache_file')
    manual_protocol_map_file = LaunchConfiguration('manual_protocol_map_file')
    lx_id_ranges = LaunchConfiguration('lx_id_ranges')
    zl_id_ranges = LaunchConfiguration('zl_id_ranges')
    probe_on_startup = LaunchConfiguration('probe_on_startup')
    probe_timeout_sec = LaunchConfiguration('probe_timeout_sec')
    probe_on_unknown_command = LaunchConfiguration('probe_on_unknown_command')
    probe_retry_interval_sec = LaunchConfiguration('probe_retry_interval_sec')
    runtime_probe_interval_sec = LaunchConfiguration('runtime_probe_interval_sec')
    read_service_timeout_sec = LaunchConfiguration('read_service_timeout_sec')
    heartbeat_debug = LaunchConfiguration('heartbeat_debug')
    ws_debug = LaunchConfiguration('ws_debug')
    debug_aggregate = LaunchConfiguration('debug_aggregate')
    debug_aggregate_period = LaunchConfiguration('debug_aggregate_period')
    debug_aggregate_max_len = LaunchConfiguration('debug_aggregate_max_len')
    bvh_action_file = LaunchConfiguration('bvh_action_file')
    enable_isaac_bridge = LaunchConfiguration('enable_isaac_bridge')
    isaac_bridge_debug = LaunchConfiguration('isaac_bridge_debug')
    isaac_command_topic = LaunchConfiguration('isaac_command_topic')
    isaac_state_topic = LaunchConfiguration('isaac_state_topic')
    isaac_enforce_limits = LaunchConfiguration('isaac_enforce_limits')
    enable_sim_cpp_bridge = LaunchConfiguration('enable_sim_cpp_bridge')
    sim_cpp_bridge_debug = LaunchConfiguration('sim_cpp_bridge_debug')
    sim_joint_cmd_topic = LaunchConfiguration('sim_joint_cmd_topic')
    sim_joint_state_fb_topic = LaunchConfiguration('sim_joint_state_fb_topic')
    sim_publish_rate_hz = LaunchConfiguration('sim_publish_rate_hz')

    pca_debug_arg = DeclareLaunchArgument(
        'pca_debug',
        default_value=LaunchConfiguration('debug'),
        description='PCA9685 舵机驱动调试模式'
    )
    pca_debug = LaunchConfiguration('pca_debug')

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
            {'debug': bridge_debug},
            {'imu_debug': imu_debug},
            {'heartbeat_debug': heartbeat_debug},
            {'ws_debug': ws_debug},
            {'debug_aggregate': debug_aggregate},
            {'debug_aggregate_period': debug_aggregate_period},
            {'debug_aggregate_max_len': debug_aggregate_max_len},
            {'bvh_action_file': bvh_action_file}
        ],
        remappings=[
            # 如果需要重新映射话题名称，可以在这里配置
        ]
    )

    # 1.1 Isaac-ROS桥接节点（可选）
    isaac_bridge_node = Node(
        package='websocket_bridge',
        executable='isaac_bridge_node',
        name='isaac_ros_bridge',
        output='screen',
        condition=IfCondition(enable_isaac_bridge),
        parameters=[
            {'isaac_command_topic': isaac_command_topic},
            {'isaac_state_topic': isaac_state_topic},
            {'servo_command_topic': '/servo/command'},
            {'servo_state_topic': '/servo/state'},
            {'enforce_position_limits': isaac_enforce_limits},
            {'debug': isaac_bridge_debug},
        ]
    )

    # 1.2 C++仿真桥接节点（可选）
    sim_cpp_bridge_node = Node(
        package='sim_servo_bridge_cpp',
        executable='sim_servo_bridge_node',
        name='sim_servo_bridge',
        output='screen',
        condition=IfCondition(enable_sim_cpp_bridge),
        parameters=[
            {'joint_cmd_topic': sim_joint_cmd_topic},
            {'joint_state_fb_topic': sim_joint_state_fb_topic},
            {'servo_cmd_topic': '/servo/command'},
            {'servo_state_topic': '/servo/state'},
            {'publish_rate_hz': sim_publish_rate_hz},
            {'speed': 100},
            {'debug': sim_cpp_bridge_debug},
        ]
    )

    # 2. 动态创建总线舵机串口驱动节点（从 bus_servo_map.json 读取）
    bus_port_nodes = []
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
            executable='bus_port_driver',
            name=f'bus_port_driver_{idx}',
            output='screen',
            parameters=[
                {'port': port},
                {'baudrate': baudrate},
                {'default_speed': 100},
                # 先将端口内ID注入两类协议集合，具体协议由router决定
                {'zl_servo_ids': servo_ids},
                {'lx_servo_ids': servo_ids},
                {'debug': bus_servo_debug},
                {'log_id': True}
            ]
        )
        bus_port_nodes.append(node)

        # 生成启动信息行
        servo_ids_str = ', '.join(map(str, servo_ids))
        servo_info_lines.append(f'    - {port} @ 115200 bps (舵机ID: {servo_ids_str})\n')

    # 2.1 总线协议路由节点（统一入口: /servo/command -> /servo/state）
    bus_protocol_router_node = Node(
        package='servo_hardware',
        executable='bus_protocol_router',
        name='bus_protocol_router',
        output='screen',
        parameters=[
            {'bus_map_file': config_file},
            {'protocol_cache_file': protocol_cache_file},
            {'manual_protocol_map_file': manual_protocol_map_file},
            {'lx_id_ranges': lx_id_ranges},
            {'zl_id_ranges': zl_id_ranges},
            {'probe_on_startup': probe_on_startup},
            {'probe_timeout_sec': probe_timeout_sec},
            {'probe_on_unknown_command': probe_on_unknown_command},
            {'probe_retry_interval_sec': probe_retry_interval_sec},
            {'runtime_probe_interval_sec': runtime_probe_interval_sec},
            {'read_service_timeout_sec': read_service_timeout_sec},
            {'probe_wait_service_sec': 6.0},
            {'debug': bus_servo_debug},
        ],
    )

    # 3. IMU 传感器串口驱动节点（使用串口而非I2C）
    imu_driver_node = Node(
        package='servo_hardware',
        executable='imu_serial_driver',  # 使用串口版本
        name='imu_serial_driver',
        output='screen',
        condition=IfCondition(imu_enable),
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
    #         {'debug': pca_debug}
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
            '  Isaac桥接启用: ', enable_isaac_bridge, '\n',
            '    命令转发: ', isaac_command_topic, ' -> /servo/command\n',
            '    状态转发: /servo/state -> ', isaac_state_topic, '\n',
            '  C++仿真桥启用: ', enable_sim_cpp_bridge, '\n',
            '    命令转发: ', sim_joint_cmd_topic, ' -> /servo/command\n',
            '    状态转发: /servo/state -> ', sim_joint_state_fb_topic, '\n',
            '  联调建议: enable_sim_cpp_bridge:=true 时设置 enable_isaac_bridge:=false\n',
            '  总线协议路由: /servo/command -> bus_protocol_router -> /bus_port_driver_x/command_{zl,lx}\n',
            '  全局读角度服务: /servo/read_position\n',
            '  全局通用指令服务: /servo/execute_command\n',
            '  协议缓存文件: ', protocol_cache_file, '\n',
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
        bridge_debug_arg,
        bus_servo_debug_arg,
        protocol_cache_file_arg,
        manual_protocol_map_file_arg,
        lx_id_ranges_arg,
        zl_id_ranges_arg,
        probe_on_startup_arg,
        probe_timeout_sec_arg,
        probe_on_unknown_command_arg,
        probe_retry_interval_sec_arg,
        runtime_probe_interval_sec_arg,
        read_service_timeout_sec_arg,
        heartbeat_debug_arg,
        ws_debug_arg,
        debug_aggregate_arg,
        debug_aggregate_period_arg,
        debug_aggregate_max_len_arg,
        bvh_action_file_arg,
        isaac_bridge_enable_arg,
        isaac_bridge_debug_arg,
        isaac_command_topic_arg,
        isaac_state_topic_arg,
        isaac_enforce_limits_arg,
        enable_sim_cpp_bridge_arg,
        sim_cpp_bridge_debug_arg,
        sim_joint_cmd_topic_arg,
        sim_joint_state_fb_topic_arg,
        sim_publish_rate_hz_arg,
        baudrate_arg,
        i2c_address_arg,
        i2c_bus_arg,
        imu_port_arg,  # IMU 串口设备
        imu_baudrate_arg,  # IMU 串口波特率
        imu_debug_arg,
        imu_publish_rate_arg,
        imu_algo_type_arg,
        imu_sensor_id_arg,
        imu_enable_arg,
        pca_debug_arg,

        # 启动信息
        log_info,

        # 节点
        bridge_node,
        isaac_bridge_node,
        sim_cpp_bridge_node,
        *bus_port_nodes,   # 动态生成的所有总线舵机串口驱动节点
        bus_protocol_router_node,
        imu_driver_node,   # IMU 传感器串口驱动节点
        # pca_servo_node,  # 已临时禁用 - I2C设备未连接
    ])
