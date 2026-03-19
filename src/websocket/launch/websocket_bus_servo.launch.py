"""
3DOF 联动启动封装 - 复用 full_system 总线链路。

启动组件:
1. full_system.launch.py（WebSocket + bus_protocol_router + bus_port_driver_*）
2. parallel_3dof_multi.launch.py

说明:
- 本文件不再维护单独的总线驱动实现，避免与 full_system 形成两套链路。
- 主要用于 LAUNCH_MODE=multi 场景，叠加 3DOF 多实例控制器。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成Launch描述"""

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

    protocol_cache_file_arg = DeclareLaunchArgument(
        'protocol_cache_file',
        default_value='/root/ros_ws/src/hardware/servo_hardware/config/bus_protocol_cache.json',
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
        description='幻尔协议ID范围(逗号分隔)'
    )

    zl_id_ranges_arg = DeclareLaunchArgument(
        'zl_id_ranges',
        default_value='35-43',
        description='众灵协议ID范围(逗号分隔)'
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

    imu_debug_arg = DeclareLaunchArgument(
        'imu_debug',
        default_value='false',
        description='IMU 是否启用调试模式'
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

    full_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('websocket_bridge'),
                'launch',
                'full_system.launch.py'
            ])
        ),
        launch_arguments={
            'debug': LaunchConfiguration('debug'),
            'bridge_debug': LaunchConfiguration('bridge_debug'),
            'bus_servo_debug': LaunchConfiguration('bus_servo_debug'),
            'imu_debug': LaunchConfiguration('imu_debug'),
            'heartbeat_debug': LaunchConfiguration('heartbeat_debug'),
            'ws_debug': LaunchConfiguration('ws_debug'),
            'debug_aggregate': LaunchConfiguration('debug_aggregate'),
            'debug_aggregate_period': LaunchConfiguration('debug_aggregate_period'),
            'debug_aggregate_max_len': LaunchConfiguration('debug_aggregate_max_len'),
            'protocol_cache_file': LaunchConfiguration('protocol_cache_file'),
            'manual_protocol_map_file': LaunchConfiguration('manual_protocol_map_file'),
            'lx_id_ranges': LaunchConfiguration('lx_id_ranges'),
            'zl_id_ranges': LaunchConfiguration('zl_id_ranges'),
            'probe_on_startup': LaunchConfiguration('probe_on_startup'),
            'probe_timeout_sec': LaunchConfiguration('probe_timeout_sec'),
            'probe_on_unknown_command': LaunchConfiguration('probe_on_unknown_command'),
            'probe_retry_interval_sec': LaunchConfiguration('probe_retry_interval_sec'),
            'runtime_probe_interval_sec': LaunchConfiguration('runtime_probe_interval_sec'),
            'read_service_timeout_sec': LaunchConfiguration('read_service_timeout_sec'),
            # multi 模式保持轻量，不自动启 Isaac/仿真桥
            'enable_isaac_bridge': 'false',
            'enable_sim_cpp_bridge': 'false',
        }.items()
    )

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
        heartbeat_debug_arg,
        ws_debug_arg,
        debug_aggregate_arg,
        debug_aggregate_period_arg,
        debug_aggregate_max_len_arg,
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
        imu_debug_arg,
        instances_file_arg,
        full_system,
        parallel_3dof_multi,
    ])
