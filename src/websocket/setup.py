from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'websocket_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 安装配置文件
        (os.path.join('share', package_name, 'config'),
            glob('config/*.json') + glob('config/*.md')),
    ],
    install_requires=['setuptools', 'websockets', 'rclpy'],
    zip_safe=True,
    maintainer='chenpeel',
    maintainer_email='chenpeel@foxmail.com',
    description='WebSocket通信桥接服务器，用于远程舵机控制',
    license='BSD-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ws_server=websocket_bridge.ws_server:main',
            'bridge_node=websocket_bridge.bridge_node:main',
        ],
    },
)
