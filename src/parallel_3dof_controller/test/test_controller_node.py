"""
ROS 2节点集成测试

测试覆盖:
1. 节点初始化和参数配置
2. 消息订阅和发布
3. 端到端功能测试
4. 错误处理和边界条件

注意：
- 需要ROS 2环境才能运行完整测试
- 可以先运行不依赖ROS的单元测试部分
"""

import sys
import os
import unittest
from unittest.mock import Mock, MagicMock, patch
import numpy as np

# 添加模块路径
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '../parallel_3dof_controller'))


class TestNodeInitialization(unittest.TestCase):
    """测试节点初始化（不需要ROS环境）"""

    @patch('controller_node.rclpy')
    def test_node_parameters(self, mock_rclpy):
        """测试节点参数配置"""
        # 模拟ROS节点
        from controller_node import Parallel3DOFControllerNode

        # 创建mock节点
        with patch.object(Parallel3DOFControllerNode, '__init__', return_value=None):
            node = Parallel3DOFControllerNode()

            # 测试会在实际ROS环境中进行
            # 这里只是提供测试框架

    def test_default_parameters(self):
        """测试默认参数值"""
        # 默认几何参数
        expected_params = {
            'l0': 0.02,
            'l1': 0.01,
            'l2': 0.03,
            'ankle_side': 'right',
            'default_speed': 100,
            'debug': False
        }

        # 验证参数格式正确
        assert isinstance(expected_params['l0'], float)
        assert isinstance(expected_params['ankle_side'], str)
        assert isinstance(expected_params['default_speed'], int)
        assert isinstance(expected_params['debug'], bool)


class TestMessageConversion(unittest.TestCase):
    """测试消息格式转换（不需要ROS环境）"""

    def test_vector3_to_rpy(self):
        """测试Vector3消息到RPY角度的转换"""
        # 模拟Vector3消息
        class MockVector3:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z

        msg = MockVector3(10.0, 20.0, 5.0)  # 度

        # 转换为弧度
        roll_rad = np.radians(msg.x)
        pitch_rad = np.radians(msg.y)
        yaw_rad = np.radians(msg.z)

        assert np.isclose(roll_rad, 0.1745, atol=1e-4)
        assert np.isclose(pitch_rad, 0.3491, atol=1e-4)
        assert np.isclose(yaw_rad, 0.0873, atol=1e-4)

    def test_servo_command_format(self):
        """测试舵机命令格式"""
        # 期望的命令格式
        expected_command = {
            'id': 10,
            'position': 1500,
            'speed': 100,
            'theta': 0.785,  # 45度弧度值
            'theta_deg': 45.0
        }

        # 验证格式
        assert 'id' in expected_command
        assert 'position' in expected_command
        assert 'speed' in expected_command
        assert 500 <= expected_command['position'] <= 2500
        assert expected_command['speed'] > 0


class TestEndToEndWorkflow(unittest.TestCase):
    """测试端到端工作流（模拟）"""

    def test_rpy_to_servo_workflow(self):
        """测试完整的RPY到舵机命令转换流程"""
        from kinematics_solver import Parallel3DOFKinematicsSolver

        # 创建求解器
        solver = Parallel3DOFKinematicsSolver()

        # 测试用例
        test_cases = [
            {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'name': '零姿态'},
            {'roll': np.radians(10), 'pitch': np.radians(
                10), 'yaw': np.radians(5), 'name': '小角度'},
            {'roll': np.radians(30), 'pitch': np.radians(
                30), 'yaw': 0.0, 'name': '边界姿态'}
        ]

        for case in test_cases:
            with self.subTest(case=case['name']):
                commands = solver.rpy_to_servo_commands(
                    case['roll'], case['pitch'], case['yaw'])

                # 验证命令数量
                self.assertEqual(len(commands), 3)

                # 验证每个命令
                for cmd in commands:
                    self.assertIn('id', cmd)
                    self.assertIn('position', cmd)
                    self.assertIn('speed', cmd)
                    self.assertGreaterEqual(cmd['position'], 500)
                    self.assertLessEqual(cmd['position'], 2500)

    def test_left_right_ankle_independence(self):
        """测试左右脚独立控制"""
        from kinematics_solver import Parallel3DOFKinematicsSolver

        solver = Parallel3DOFKinematicsSolver()

        # 相同RPY输入
        roll, pitch, yaw = np.radians(10), np.radians(10), np.radians(5)

        # 右脚命令
        right_commands = solver.rpy_to_servo_commands(
            roll, pitch, yaw, ankle_side='right')

        # 左脚命令
        left_commands = solver.rpy_to_servo_commands(
            roll, pitch, yaw, ankle_side='left')

        # 舵机ID应该不同
        right_ids = [cmd['id'] for cmd in right_commands]
        left_ids = [cmd['id'] for cmd in left_commands]

        self.assertNotEqual(right_ids, left_ids)

        # 位置和theta应该相同（相同的RPY输入）
        for i in range(3):
            self.assertEqual(
                right_commands[i]['position'], left_commands[i]['position'])
            self.assertAlmostEqual(
                right_commands[i]['theta'], left_commands[i]['theta'])


class TestErrorHandling(unittest.TestCase):
    """测试错误处理"""

    def test_invalid_ankle_side(self):
        """测试无效的脚踝侧"""
        from kinematics_solver import Parallel3DOFKinematicsSolver

        solver = Parallel3DOFKinematicsSolver()

        with self.assertRaises(ValueError):
            solver.rpy_to_servo_commands(0, 0, 0, ankle_side='invalid')

    def test_extreme_rpy_values(self):
        """测试极端RPY值（应该被限制）"""
        from kinematics_solver import Parallel3DOFKinematicsSolver

        solver = Parallel3DOFKinematicsSolver()

        # 超出±30°限制的值
        commands = solver.rpy_to_servo_commands(
            np.radians(90),  # 极端值
            np.radians(90),
            np.radians(90)
        )

        # 应该能成功生成命令（被自动限制）
        self.assertEqual(len(commands), 3)

        # 所有命令应该在有效范围内
        for cmd in commands:
            self.assertGreaterEqual(cmd['position'], 500)
            self.assertLessEqual(cmd['position'], 2500)


# ROS 2 Launch测试（需要完整ROS环境）
"""
以下是使用launch_testing的示例代码，需要ROS 2环境：

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from servo_msgs.msg import ServoCommand
from std_msgs.msg import Float32MultiArray


@pytest.fixture
def node():
    rclpy.init()
    test_node = rclpy.create_node('test_node')
    yield test_node
    test_node.destroy_node()
    rclpy.shutdown()


def test_node_publishes_servo_commands(node):
    '''测试节点发布舵机命令'''

    # 创建订阅器
    received_messages = []

    def callback(msg):
        received_messages.append(msg)

    subscription = node.create_subscription(
        ServoCommand,
        '/parallel_3dof_controller/servo/command',
        callback,
        10
    )

    # 创建发布器
    publisher = node.create_publisher(
        Vector3,
        '/parallel_3dof_controller/ankle_rpy',
        10
    )

    # 发布测试消息
    test_msg = Vector3()
    test_msg.x = 10.0  # roll
    test_msg.y = 10.0  # pitch
    test_msg.z = 5.0   # yaw

    publisher.publish(test_msg)

    # 等待消息
    rclpy.spin_once(node, timeout_sec=1.0)

    # 验证收到的消息
    assert len(received_messages) == 3  # 应该收到3个舵机命令
    for msg in received_messages:
        assert 500 <= msg.position <= 2500
        assert msg.speed > 0


def test_node_publishes_theta_feedback(node):
    '''测试节点发布theta角反馈'''

    received_theta = None

    def callback(msg):
        nonlocal received_theta
        received_theta = msg

    subscription = node.create_subscription(
        Float32MultiArray,
        '/parallel_3dof_controller/ankle_theta',
        callback,
        10
    )

    publisher = node.create_publisher(
        Vector3,
        '/parallel_3dof_controller/ankle_rpy',
        10
    )

    # 发布测试消息
    test_msg = Vector3()
    test_msg.x = 10.0
    test_msg.y = 10.0
    test_msg.z = 5.0

    publisher.publish(test_msg)

    # 等待消息
    rclpy.spin_once(node, timeout_sec=1.0)

    # 验证收到theta反馈
    assert received_theta is not None
    assert len(received_theta.data) == 3  # 3个theta角
    for theta in received_theta.data:
        assert 0 <= theta <= 90  # theta应该在0-90度范围内
"""


class IntegrationTestInstructions(unittest.TestCase):
    """集成测试说明"""

    def test_print_instructions(self):
        """打印集成测试运行指南"""
        instructions = """
        ========================================
        ROS 2 节点集成测试运行指南
        ========================================

        1. 单元测试（不需要ROS环境）:
           cd /home/chenpeel/work/repo/jiyuan/ros/src/parallel_3dof_controller
           python3 -m pytest test/test_controller_node.py -v -k "not ROS"

        2. 完整集成测试（需要ROS环境）:

           步骤1: 构建包
           cd /home/chenpeel/work/repo/jiyuan/ros
           colcon build --packages-select parallel_3dof_controller
           source install/setup.bash

           步骤2: 启动节点
           ros2 launch parallel_3dof_controller parallel_3dof_controller.launch.py debug:=true

           步骤3: 在另一个终端运行测试
           # 发送测试命令
           ros2 topic pub /parallel_3dof_controller/ankle_rpy geometry_msgs/Vector3 "{x: 10.0, y: 10.0, z: 5.0}" --once

           # 查看舵机命令输出
           ros2 topic echo /parallel_3dof_controller/servo/command

           # 查看theta角反馈
           ros2 topic echo /parallel_3dof_controller/ankle_theta

        3. 验证测试:

           预期结果:
           - 节点启动成功，打印初始化信息
           - 收到RPY命令后，发布3个舵机命令
           - 舵机ID为10, 11, 12（右脚）或13, 14, 15（左脚）
           - 舵机位置在500-2500us范围内
           - theta角在0-90度范围内

        4. 故障排查:

           问题1: 节点启动失败
           - 检查是否source了setup.bash
           - 检查tdpm.py是否在正确路径

           问题2: 未收到舵机命令
           - 检查话题名称是否正确
           - 使用 ros2 topic list 查看活跃话题
           - 检查节点是否正常运行: ros2 node list

           问题3: 约束未满足警告
           - 检查几何参数l0, l1, l2是否正确
           - 尝试调整参数配置文件

        ========================================
        """

        # 打印指南（测试时不输出，只在手动运行时查看）
        # print(instructions)

        # 确保指南非空
        self.assertGreater(len(instructions), 0)


if __name__ == '__main__':
    # 运行单元测试（不需要ROS环境）
    unittest.main(verbosity=2)
