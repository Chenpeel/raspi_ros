import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'servo_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('servo_hardware/config/*.json') + glob('servo_hardware/config/*.md'),
        ),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'pyserial',  # 总线舵机串口通信
        'smbus2',    # PCA9685 I2C通信
    ],
    zip_safe=True,
    maintainer='chenpeel',
    maintainer_email='chenpeel@foxmail.com',
    description='舵机硬件接口，用于舵机控制',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bus_servo_driver = servo_hardware.bus_servo:main',
            'bus_port_driver = servo_hardware.bus_port_driver:main',
            'bus_protocol_router = servo_hardware.bus_protocol_router:main',
            'pca_servo_driver = servo_hardware.pca_servo:main',
            'servo_router = servo_hardware.servo_router:main',
            'imu_driver = sensor_hardware.imu_driver:main',  # I2C 版本
            'imu_serial_driver = sensor_hardware.imu_serial_driver:main',  # 串口版本
        ],
    },
)
