from setuptools import setup, find_packages

package_name = 'servo_drivers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenpeel',
    maintainer_email='chgenpeel@foxmail.com',
    description='ROS2舵机驱动插件',
    license='MIT',
    tests_require=['pytest'],
)
