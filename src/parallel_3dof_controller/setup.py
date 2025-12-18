from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'parallel_3dof_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='3自由度并联机构通用控制器 - 可用于脚踝、腰部等关节',
    license='MIT',
    # 测试依赖在 package.xml 中管理
    entry_points={
        'console_scripts': [
            'parallel_3dof_node = parallel_3dof_controller.controller_node:main',
        ],
    },
)
