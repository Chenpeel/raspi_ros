from setuptools import setup
import os
from glob import glob

package_name = 'record_load_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.json') + glob('config/*.md')),
        (os.path.join('share', package_name, 'config', 'bvh'),
            glob('config/bvh/*.bvh') + glob('config/bvh/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chenpeel',
    maintainer_email='chenpeel@foxmail.com',
    description='Action resources for websocket bridge (BVH and others)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bvh_static_convert=record_load_action.bvh_static_convert:main',
        ],
    },
)
