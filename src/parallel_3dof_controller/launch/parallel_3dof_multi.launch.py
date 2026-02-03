"""
Multi-instance launch for 3-DOF parallel controllers.
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_instances(context):
    instances_file = LaunchConfiguration('instances_file').perform(context)
    if not os.path.isfile(instances_file):
        raise RuntimeError(f"instances_file not found: {instances_file}")

    with open(instances_file, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}

    instances = data.get('instances', [])
    if not isinstance(instances, list):
        raise RuntimeError("instances must be a list in the yaml file")

    nodes = []
    for index, inst in enumerate(instances):
        if not isinstance(inst, dict):
            raise RuntimeError(f"instance #{index} must be a dict")

        name = inst.get('name')
        if not name:
            raise RuntimeError(f"instance #{index} missing required field: name")

        namespace = inst.get('namespace', '')
        params = inst.get('params', {}) or {}
        if not isinstance(params, dict):
            raise RuntimeError(f"instance #{index} params must be a dict")

        nodes.append(
            Node(
                package='parallel_3dof_controller',
                executable='parallel_3dof_node',
                name=name,
                namespace=namespace,
                parameters=[params],
                output='screen',
                emulate_tty=True,
            )
        )

    return nodes


def generate_launch_description():
    pkg_share = FindPackageShare('parallel_3dof_controller')
    default_instances_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'parallel_3dof_instances.yaml'
    ])

    instances_file_arg = DeclareLaunchArgument(
        'instances_file',
        default_value=default_instances_file,
        description='YAML file that defines controller instances'
    )

    return LaunchDescription([
        instances_file_arg,
        OpaqueFunction(function=_load_instances),
    ])
