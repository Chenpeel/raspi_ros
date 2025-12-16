from setuptools import find_packages, setup

package_name = 'servo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'websocket_bridge', 'servo_core'],
    zip_safe=True,
    maintainer='chenpeel',
    maintainer_email='chgenpeel@foxmail.com',
    description='舵机控制节点，整合WebSocket和ROS2',
    license='BSD-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'servo_control_node=servo_control.servo_control_node:main',
        ],
    },
)
