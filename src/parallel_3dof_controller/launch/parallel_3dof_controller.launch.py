"""
3-DOF并联控制器Launch文件

启动3-DOF并联控制器节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成Launch描述"""

    # 获取包的共享目录
    pkg_share = FindPackageShare('parallel_3dof_controller')

    # 配置文件路径
    default_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'parallel_3dof_params.yaml'
    ])

    # 声明Launch参数
    ankle_side_arg = DeclareLaunchArgument(
        'ankle_side',
        default_value='right',
        description='控制哪侧脚踝 (right/left)'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='是否启用调试模式'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='配置文件路径'
    )

    # 3-DOF并联控制器节点
    parallel_3dof_controller_node = Node(
        package='parallel_3dof_controller',
        executable='parallel_3dof_node',  # 修正：与 setup.py 中的 entry_point 一致
        name='parallel_3dof_controller',
        namespace='',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'ankle_side': LaunchConfiguration('ankle_side'),
                'debug': LaunchConfiguration('debug'),
            }
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            # 将私有话题映射到全局话题
            ('~/servo/command', '/servo/command'),
        ]
    )

    return LaunchDescription([
        ankle_side_arg,
        debug_arg,
        config_file_arg,
        parallel_3dof_controller_node,
    ])
