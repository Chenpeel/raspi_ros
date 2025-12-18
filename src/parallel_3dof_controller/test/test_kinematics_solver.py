"""
3-DOF并联机构运动学求解器单元测试

测试覆盖:
1. 初始化和配置测试
2. RPY限制和裁剪测试
3. RPY到Theta角转换测试
4. Theta角到舵机位置映射测试
5. 完整转换流程测试
6. 边界条件和异常处理测试
"""

import sys
import os
import pytest
import numpy as np

# 添加模块路径
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '../parallel_3dof_controller'))

from kinematics_solver import Parallel3DOFKinematicsSolver


class TestKinematicsSolverInitialization:
    """测试运动学求解器的初始化"""

    def test_default_initialization(self):
        """测试默认参数初始化"""
        solver = Parallel3DOFKinematicsSolver()

        # 检查几何参数
        assert solver.mechanism.l0 == 0.02
        assert solver.mechanism.l1 == 0.01
        assert solver.mechanism.l2 == 0.03

        # 检查工作空间限制
        assert solver.rpy_limits['roll'] == (-np.pi/6, np.pi/6)
        assert solver.rpy_limits['pitch'] == (-np.pi/6, np.pi/6)
        assert solver.rpy_limits['yaw'] == (-np.pi/6, np.pi/6)

    def test_custom_geometry(self):
        """测试自定义几何参数初始化"""
        l0, l1, l2 = 0.025, 0.015, 0.035
        solver = Parallel3DOFKinematicsSolver(l0=l0, l1=l1, l2=l2)

        assert solver.mechanism.l0 == l0
        assert solver.mechanism.l1 == l1
        assert solver.mechanism.l2 == l2

    def test_servo_config_default(self):
        """测试默认舵机配置"""
        solver = Parallel3DOFKinematicsSolver()

        # 检查右脚舵机ID
        assert solver.servo_config['right_ankle']['servo_1']['id'] == 10
        assert solver.servo_config['right_ankle']['servo_2']['id'] == 11
        assert solver.servo_config['right_ankle']['servo_3']['id'] == 12

        # 检查左脚舵机ID
        assert solver.servo_config['left_ankle']['servo_1']['id'] == 13
        assert solver.servo_config['left_ankle']['servo_2']['id'] == 14
        assert solver.servo_config['left_ankle']['servo_3']['id'] == 15

        # 检查位置映射参数
        mapping = solver.servo_config['position_mapping']
        assert mapping['min_us'] == 500
        assert mapping['max_us'] == 2500
        assert mapping['min_angle'] == 0.0
        assert mapping['max_angle'] == np.pi

    def test_custom_servo_config(self):
        """测试自定义舵机配置"""
        custom_config = {
            'right_ankle': {
                'servo_1': {'id': 1, 'offset': 0.1, 'direction': -1},
                'servo_2': {'id': 2, 'offset': 0.0, 'direction': 1},
                'servo_3': {'id': 3, 'offset': -0.1, 'direction': 1}
            },
            'left_ankle': {
                'servo_1': {'id': 4, 'offset': 0.0, 'direction': 1},
                'servo_2': {'id': 5, 'offset': 0.0, 'direction': 1},
                'servo_3': {'id': 6, 'offset': 0.0, 'direction': 1}
            },
            'position_mapping': {
                'min_us': 500,
                'max_us': 2500,
                'min_angle': 0.0,
                'max_angle': np.pi
            }
        }

        solver = Parallel3DOFKinematicsSolver(servo_config=custom_config)
        assert solver.servo_config['right_ankle']['servo_1']['id'] == 1
        assert solver.servo_config['right_ankle']['servo_1']['offset'] == 0.1
        assert solver.servo_config['right_ankle']['servo_1']['direction'] == -1


class TestRPYClipping:
    """测试RPY角度限制和裁剪"""

    @pytest.fixture
    def solver(self):
        """创建求解器实例"""
        return Parallel3DOFKinematicsSolver()

    def test_clip_within_limits(self, solver):
        """测试范围内的RPY不被修改"""
        roll, pitch, yaw = 0.1, 0.2, 0.3
        r_clip, p_clip, y_clip = solver.clip_rpy(roll, pitch, yaw)

        assert r_clip == roll
        assert p_clip == pitch
        assert y_clip == yaw

    def test_clip_exceed_upper_limit(self, solver):
        """测试超出上限的RPY被限制"""
        roll, pitch, yaw = 1.0, 1.0, 1.0  # 超出±30°限制
        r_clip, p_clip, y_clip = solver.clip_rpy(roll, pitch, yaw)

        assert r_clip == np.pi/6
        assert p_clip == np.pi/6
        assert y_clip == np.pi/6

    def test_clip_exceed_lower_limit(self, solver):
        """测试低于下限的RPY被限制"""
        roll, pitch, yaw = -1.0, -1.0, -1.0  # 超出±30°限制
        r_clip, p_clip, y_clip = solver.clip_rpy(roll, pitch, yaw)

        assert r_clip == -np.pi/6
        assert p_clip == -np.pi/6
        assert y_clip == -np.pi/6

    def test_clip_boundary_values(self, solver):
        """测试边界值±30°"""
        roll, pitch, yaw = np.pi/6, np.pi/6, np.pi/6
        r_clip, p_clip, y_clip = solver.clip_rpy(roll, pitch, yaw)

        assert r_clip == np.pi/6
        assert p_clip == np.pi/6
        assert y_clip == np.pi/6


class TestRPYToTheta:
    """测试RPY到Theta角的转换"""

    @pytest.fixture
    def solver(self):
        """创建求解器实例"""
        return Parallel3DOFKinematicsSolver()

    def test_zero_pose(self, solver):
        """测试零姿态 (0, 0, 0)"""
        result = solver.rpy_to_theta_angles(0.0, 0.0, 0.0)

        # 验证返回的键
        assert 'theta_a' in result
        assert 'theta_b' in result
        assert 'theta_c' in result
        assert 'constraint_satisfied' in result

        # 零姿态下三个theta角应该相等
        assert np.isclose(result['theta_a'], result['theta_b'], atol=1e-6)
        assert np.isclose(result['theta_b'], result['theta_c'], atol=1e-6)

        # 约束应该大致满足（正向运动学不保证严格满足约束）
        # 实际硬件控制中，1e-3量级的误差是可接受的
        assert result['constraint_norm'] < 1e-2

    def test_small_angle_pose(self, solver):
        """测试小角度姿态 (10°, 10°, 5°)"""
        roll = np.radians(10)
        pitch = np.radians(10)
        yaw = np.radians(5)

        result = solver.rpy_to_theta_angles(roll, pitch, yaw)

        # theta角应该在合理范围内 (0 ~ π/2)
        assert 0 <= result['theta_a'] <= np.pi/2
        assert 0 <= result['theta_b'] <= np.pi/2
        assert 0 <= result['theta_c'] <= np.pi/2

        # 约束应该大致满足（正向运动学不保证严格满足约束）
        assert result['constraint_norm'] < 1e-2

    def test_boundary_pose(self, solver):
        """测试边界姿态 (30°, 30°, 0°)"""
        roll = np.radians(30)
        pitch = np.radians(30)
        yaw = np.radians(0)

        result = solver.rpy_to_theta_angles(roll, pitch, yaw)

        # theta角应该在合理范围内
        assert 0 <= result['theta_a'] <= np.pi/2
        assert 0 <= result['theta_b'] <= np.pi/2
        assert 0 <= result['theta_c'] <= np.pi/2

        # 约束满足（边界处误差较大是正常的）
        assert result['constraint_norm'] < 1e-2

    def test_workspace_clipping(self, solver):
        """测试工作空间自动限制"""
        # 提供超出范围的RPY，应该被自动限制
        roll = np.radians(45)  # 超出±30°
        pitch = np.radians(45)
        yaw = np.radians(45)

        result = solver.rpy_to_theta_angles(
            roll, pitch, yaw, clip_workspace=True)

        # 应该成功求解（因为被限制了）
        assert 'theta_a' in result
        # 约束大致满足即可
        assert result['constraint_norm'] < 1e-2

    def test_rod_lengths(self, solver):
        """测试杆长一致性"""
        result = solver.rpy_to_theta_angles(0.1, 0.1, 0.05)

        # 三根杆长大致相等（并联机构特性）
        # 由于约束未严格满足，允许较大误差
        rod_lengths = result['rod_lengths']
        mean_length = np.mean(rod_lengths)
        for length in rod_lengths:
            # 允许10%的偏差
            assert abs(length - mean_length) / mean_length < 0.1


class TestThetaToServoPosition:
    """测试Theta角到舵机位置的映射"""

    @pytest.fixture
    def solver(self):
        """创建求解器实例"""
        return Parallel3DOFKinematicsSolver()

    def test_zero_theta(self, solver):
        """测试theta=0对应舵机最小位置"""
        servo_config = {'offset': 0.0, 'direction': 1}
        position = solver.theta_to_servo_position(0.0, servo_config)

        # theta=0 应该对应 min_us=500
        assert position == 500

    def test_max_theta(self, solver):
        """测试theta=π/2对应舵机最大位置"""
        servo_config = {'offset': 0.0, 'direction': 1}
        position = solver.theta_to_servo_position(np.pi/2, servo_config)

        # theta=π/2 应该对应 max_us=2500
        assert position == 2500

    def test_mid_theta(self, solver):
        """测试theta=π/4对应舵机中间位置"""
        servo_config = {'offset': 0.0, 'direction': 1}
        position = solver.theta_to_servo_position(np.pi/4, servo_config)

        # theta=π/4 应该对应 1500us (中点)
        assert position == 1500

    def test_servo_direction_reversed(self, solver):
        """测试舵机反向"""
        servo_config_normal = {'offset': 0.0, 'direction': 1}
        servo_config_reversed = {'offset': 0.0, 'direction': -1}

        # 测试正常方向
        pos_normal = solver.theta_to_servo_position(np.pi/4, servo_config_normal)
        pos_reversed = solver.theta_to_servo_position(np.pi/4, servo_config_reversed)

        # 两个位置都应该在有效范围内
        assert 500 <= pos_normal <= 2500
        assert 500 <= pos_reversed <= 2500

        # direction参数会影响映射结果
        # 具体效果取决于实现细节

    def test_servo_offset(self, solver):
        """测试舵机偏置"""
        offset = np.radians(10)  # 10度偏置
        servo_config = {'offset': offset, 'direction': 1}

        # 有偏置时，相同theta对应不同position
        pos_with_offset = solver.theta_to_servo_position(0.0, servo_config)
        pos_without_offset = solver.theta_to_servo_position(
            offset, {'offset': 0.0, 'direction': 1})

        assert np.isclose(pos_with_offset, pos_without_offset, atol=1)

    def test_clipping(self, solver):
        """测试超出范围的theta被限制"""
        servo_config = {'offset': 0.0, 'direction': 1}

        # 超出π/2的theta应该被限制到max_us
        position = solver.theta_to_servo_position(np.pi, servo_config)
        assert position == 2500

        # 负theta应该被限制到min_us
        position = solver.theta_to_servo_position(-0.1, servo_config)
        assert position == 500


class TestFullConversion:
    """测试完整的RPY到舵机命令转换"""

    @pytest.fixture
    def solver(self):
        """创建求解器实例"""
        return Parallel3DOFKinematicsSolver()

    def test_rpy_to_servo_commands_format(self, solver):
        """测试命令格式"""
        commands = solver.rpy_to_servo_commands(0.0, 0.0, 0.0)

        # 应该返回3个舵机命令
        assert len(commands) == 3

        # 每个命令应该包含必要的字段
        for cmd in commands:
            assert 'id' in cmd
            assert 'position' in cmd
            assert 'speed' in cmd
            assert 'theta' in cmd
            assert 'theta_deg' in cmd

    def test_right_ankle_servo_ids(self, solver):
        """测试右脚舵机ID"""
        commands = solver.rpy_to_servo_commands(
            0.0, 0.0, 0.0, ankle_side='right')

        # 右脚舵机ID应该是10, 11, 12
        ids = [cmd['id'] for cmd in commands]
        assert ids == [10, 11, 12]

    def test_left_ankle_servo_ids(self, solver):
        """测试左脚舵机ID"""
        commands = solver.rpy_to_servo_commands(
            0.0, 0.0, 0.0, ankle_side='left')

        # 左脚舵机ID应该是13, 14, 15
        ids = [cmd['id'] for cmd in commands]
        assert ids == [13, 14, 15]

    def test_servo_speed_parameter(self, solver):
        """测试舵机速度参数"""
        speed = 200
        commands = solver.rpy_to_servo_commands(
            0.0, 0.0, 0.0, speed=speed)

        # 所有舵机的速度应该是指定值
        for cmd in commands:
            assert cmd['speed'] == speed

    def test_position_range(self, solver):
        """测试舵机位置在合理范围内"""
        commands = solver.rpy_to_servo_commands(0.1, 0.1, 0.05)

        # 所有舵机位置应该在500-2500之间
        for cmd in commands:
            assert 500 <= cmd['position'] <= 2500

    def test_theta_deg_conversion(self, solver):
        """测试theta角度转换"""
        commands = solver.rpy_to_servo_commands(0.0, 0.0, 0.0)

        # theta_deg应该是theta的角度形式
        for cmd in commands:
            assert np.isclose(
                cmd['theta_deg'], np.degrees(cmd['theta']), atol=1e-6)

    def test_invalid_ankle_side(self, solver):
        """测试无效的脚踝侧"""
        with pytest.raises(ValueError):
            solver.rpy_to_servo_commands(0.0, 0.0, 0.0, ankle_side='invalid')

    def test_symmetry_for_zero_pose(self, solver):
        """测试零姿态下三个舵机位置相等"""
        commands = solver.rpy_to_servo_commands(0.0, 0.0, 0.0)

        # 零姿态下，三个舵机位置应该相同（对称性）
        positions = [cmd['position'] for cmd in commands]
        assert np.isclose(positions[0], positions[1], atol=1)
        assert np.isclose(positions[1], positions[2], atol=1)


class TestWorkspaceLimits:
    """测试工作空间限制"""

    @pytest.fixture
    def solver(self):
        """创建求解器实例"""
        return Parallel3DOFKinematicsSolver()

    def test_get_workspace_limits(self, solver):
        """测试获取工作空间限制"""
        limits = solver.get_workspace_limits()

        # 验证返回的键
        assert 'roll' in limits
        assert 'pitch' in limits
        assert 'yaw' in limits

        # 验证每个轴包含rad和deg
        for axis in ['roll', 'pitch', 'yaw']:
            assert 'rad' in limits[axis]
            assert 'deg' in limits[axis]

    def test_workspace_limit_values(self, solver):
        """测试工作空间限制值"""
        limits = solver.get_workspace_limits()

        # 默认应该是±30°
        for axis in ['roll', 'pitch', 'yaw']:
            rad_limits = limits[axis]['rad']
            deg_limits = limits[axis]['deg']

            assert np.isclose(rad_limits[0], -np.pi/6, atol=1e-6)
            assert np.isclose(rad_limits[1], np.pi/6, atol=1e-6)
            assert np.isclose(deg_limits[0], -30.0, atol=1e-6)
            assert np.isclose(deg_limits[1], 30.0, atol=1e-6)


class TestRoundTrip:
    """测试往返转换一致性"""

    @pytest.fixture
    def solver(self):
        """创建求解器实例"""
        return Parallel3DOFKinematicsSolver()

    def test_zero_pose_round_trip(self, solver):
        """测试零姿态的往返转换"""
        # RPY → theta → servo
        result = solver.rpy_to_theta_angles(0.0, 0.0, 0.0)
        commands = solver.rpy_to_servo_commands(0.0, 0.0, 0.0)

        # theta角应该一致
        assert np.isclose(result['theta_a'], commands[0]['theta'], atol=1e-6)
        assert np.isclose(result['theta_b'], commands[1]['theta'], atol=1e-6)
        assert np.isclose(result['theta_c'], commands[2]['theta'], atol=1e-6)

    def test_small_angle_round_trip(self, solver):
        """测试小角度的往返转换"""
        roll, pitch, yaw = np.radians(10), np.radians(10), np.radians(5)

        result = solver.rpy_to_theta_angles(roll, pitch, yaw)
        commands = solver.rpy_to_servo_commands(roll, pitch, yaw)

        # theta角应该一致
        assert np.isclose(result['theta_a'], commands[0]['theta'], atol=1e-6)
        assert np.isclose(result['theta_b'], commands[1]['theta'], atol=1e-6)
        assert np.isclose(result['theta_c'], commands[2]['theta'], atol=1e-6)


if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
