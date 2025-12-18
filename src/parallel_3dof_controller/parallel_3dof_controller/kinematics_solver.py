"""
3-DOF并联机构运动学求解器

基于3-DOF并联机构的运动学模型,实现从RPY姿态角到舵机角度的转换
"""

import sys
import numpy as np
from typing import Dict, Tuple, Optional, List

try:
    from .tdpm import ThreeDOFParallelMechanism
except ImportError:
    # 如果作为脚本直接运行,尝试相对导入
    try:
        from tdpm import ThreeDOFParallelMechanism
    except ImportError as e:
        print(f"警告: 无法导入tdpm运动学库: {e}")
        print("请确保tdpm.py在当前目录或PYTHONPATH中")
        ThreeDOFParallelMechanism = None


class Parallel3DOFKinematicsSolver:
    """
    3-DOF并联机构运动学求解器

    功能:
    - 将脚踝RPY姿态角转换为3个舵机的目标角度
    - 支持左右脚踝的独立控制
    - 提供舵机角度到位置值的映射

    设计假设:
    - 舵机角度直接对应theta角（连杆与β平面的夹角）
    - theta角范围: 0 ~ π/2 (0° ~ 90°)
    - 舵机位置范围: 500-2500us 对应 0-180°

    参数:
        l0: 平台半径 (米)
        l1: 动平台到O点的距离 (米)
        l2: 静平台到O点的距离 (米)
        servo_config: 舵机配置字典
    """

    def __init__(self,
                 l0: float = 0.02,
                 l1: float = 0.01,
                 l2: float = 0.03,
                 servo_config: Optional[Dict] = None):
        """
        初始化3-DOF并联机构运动学求解器

        参数:
            l0: 平台半径 (默认20mm)
            l1: 动平台距离 (默认10mm)
            l2: 静平台距离 (默认30mm)
            servo_config: 舵机配置
        """
        if ThreeDOFParallelMechanism is None:
            raise ImportError("无法导入ThreeDOFParallelMechanism,请检查kinematics模块")

        # 初始化运动学模型
        self.mechanism = ThreeDOFParallelMechanism(l0, l1, l2)

        # 舵机配置
        self.servo_config = servo_config or self._default_servo_config()

        # 工作空间限制 (弧度)
        self.rpy_limits = {
            'roll': (-np.pi/6, np.pi/6),   # ±30°
            'pitch': (-np.pi/6, np.pi/6),  # ±30°
            'yaw': (-np.pi/6, np.pi/6)     # ±30°
        }

        print(f"3-DOF并联机构运动学求解器初始化完成")
        print(f"  几何参数: l0={l0}m, l1={l1}m, l2={l2}m")
        print(
            f"  理论杆长: {self.mechanism.compute_theoretical_rod_length():.4f}m")

    @staticmethod
    def _default_servo_config() -> Dict:
        """默认舵机配置"""
        return {
            # 右脚踝舵机ID
            'right_ankle': {
                'servo_1': {'id': 12, 'offset': 0.0, 'direction': 1},
                'servo_2': {'id': 13, 'offset': 0.0, 'direction': 1},
                'servo_3': {'id': 14, 'offset': 0.0, 'direction': 1}
            },
            # 左脚踝舵机ID
            'left_ankle': {
                'servo_1': {'id': 9, 'offset': 0.0, 'direction': 1},
                'servo_2': {'id': 10, 'offset': 0.0, 'direction': 1},
                'servo_3': {'id': 11, 'offset': 0.0, 'direction': 1}
            },
            # 舵机位置映射参数
            'position_mapping': {
                'min_us': 500,     # 最小脉宽 (us)
                'max_us': 2500,    # 最大脉宽 (us)
                'min_angle': 0.0,  # 最小角度 (弧度)
                'max_angle': np.pi  # 最大角度 (弧度, 180°)
            }
        }

    def clip_rpy(self, roll: float, pitch: float, yaw: float) -> Tuple[float, float, float]:
        """
        限制RPY角度在安全工作空间内

        参数:
            roll: 横滚角 (弧度)
            pitch: 俯仰角 (弧度)
            yaw: 偏航角 (弧度)

        返回:
            (roll_clipped, pitch_clipped, yaw_clipped)
        """
        roll = np.clip(roll, *self.rpy_limits['roll'])
        pitch = np.clip(pitch, *self.rpy_limits['pitch'])
        yaw = np.clip(yaw, *self.rpy_limits['yaw'])
        return roll, pitch, yaw

    def rpy_to_theta_angles(self,
                            roll: float,
                            pitch: float,
                            yaw: float,
                            clip_workspace: bool = True) -> Dict:
        """
        将RPY姿态角转换为连杆theta角

        这是核心转换函数,使用tdpm的正向运动学求解

        参数:
            roll: 横滚角 (弧度)
            pitch: 俯仰角 (弧度)
            yaw: 偏航角 (弧度)
            clip_workspace: 是否自动限制在工作空间内

        返回:
            包含theta_a, theta_b, theta_c和其他运动学结果的字典

        抛出:
            ValueError: RPY超出工作空间范围
        """
        # 限制工作空间
        if clip_workspace:
            roll, pitch, yaw = self.clip_rpy(roll, pitch, yaw)

        # 构造姿态角向量
        mu = np.array([roll, pitch, yaw])

        # 调用正向运动学
        result = self.mechanism.forward_kinematics(mu)

        # 提取theta角 (连杆与β平面的夹角)
        theta_a = result['theta_a']
        theta_b = result['theta_b']
        theta_c = result['theta_c']

        # 验证约束是否满足
        constraint_check = self.mechanism.verify_constraints(mu)
        if not constraint_check['satisfied']:
            print(f"警告: 约束未完全满足, 误差={constraint_check['constraint_norm']:.2e}")

        return {
            'theta_a': theta_a,
            'theta_b': theta_b,
            'theta_c': theta_c,
            'phi_a': result['phi_a'],
            'phi_b': result['phi_b'],
            'phi_c': result['phi_c'],
            'rod_lengths': result['rod_lengths'],
            'constraint_satisfied': constraint_check['satisfied'],
            'constraint_norm': constraint_check['constraint_norm']
        }

    def theta_to_servo_position(self,
                                theta: float,
                                servo_config: Dict) -> int:
        """
        将theta角转换为舵机位置值 (脉宽us)

        映射关系:
        - theta范围: 0 ~ π/2 (0° ~ 90°)
        - 舵机范围: min_us ~ max_us (默认500-2500us)

        考虑:
        - 舵机偏置 (offset)
        - 舵机方向 (direction: 1正向, -1反向)

        参数:
            theta: 连杆角度 (弧度)
            servo_config: 舵机配置字典 {'offset', 'direction'}

        返回:
            舵机位置值 (us)
        """
        mapping = self.servo_config['position_mapping']

        # 应用偏置和方向
        theta_adjusted = theta * \
            servo_config['direction'] + servo_config['offset']

        # 线性映射到舵机范围
        # theta: [0, π/2] → position: [min_us, max_us]
        theta_range = np.pi / 2  # 0 ~ 90度
        position_range = mapping['max_us'] - mapping['min_us']

        position = mapping['min_us'] + \
            (theta_adjusted / theta_range) * position_range

        # 限制在舵机范围内
        position = np.clip(position, mapping['min_us'], mapping['max_us'])

        return int(position)

    def rpy_to_servo_commands(self,
                              roll: float,
                              pitch: float,
                              yaw: float,
                              ankle_side: str = 'right',
                              speed: int = 100) -> List[Dict]:
        """
        将RPY姿态角转换为舵机控制命令

        这是完整的转换流程:
        RPY → theta角 → 舵机位置 → 舵机命令

        参数:
            roll: 横滚角 (弧度)
            pitch: 俯仰角 (弧度)
            yaw: 偏航角 (弧度)
            ankle_side: 'right' 或 'left'
            speed: 舵机运动时间 (毫秒)

        返回:
            舵机命令列表 [{'id': int, 'position': int, 'speed': int}, ...]

        示例:
            >>> solver = Parallel3DOFKinematicsSolver()
            >>> commands = solver.rpy_to_servo_commands(0.1, 0.1, 0.0)
            >>> print(commands)
            [{'id': 10, 'position': 1500, 'speed': 100}, ...]
        """
        # 1. 将RPY转换为theta角
        kinematics_result = self.rpy_to_theta_angles(roll, pitch, yaw)

        theta_angles = [
            kinematics_result['theta_a'],
            kinematics_result['theta_b'],
            kinematics_result['theta_c']
        ]

        # 2. 获取舵机配置
        ankle_key = f'{ankle_side}_ankle'
        if ankle_key not in self.servo_config:
            raise ValueError(f"未知的脚踝侧: {ankle_side}")

        servo_configs = [
            self.servo_config[ankle_key]['servo_1'],
            self.servo_config[ankle_key]['servo_2'],
            self.servo_config[ankle_key]['servo_3']
        ]

        # 3. 转换为舵机命令
        commands = []
        for i, (theta, config) in enumerate(zip(theta_angles, servo_configs)):
            position = self.theta_to_servo_position(theta, config)
            commands.append({
                'id': config['id'],
                'position': position,
                'speed': speed,
                'theta': float(theta),  # 调试信息
                'theta_deg': float(np.degrees(theta))  # 调试信息
            })

        return commands

    def get_workspace_limits(self) -> Dict:
        """
        获取工作空间限制

        返回:
            RPY范围的字典 (弧度和角度)
        """
        return {
            'roll': {
                'rad': self.rpy_limits['roll'],
                'deg': (np.degrees(self.rpy_limits['roll'][0]),
                        np.degrees(self.rpy_limits['roll'][1]))
            },
            'pitch': {
                'rad': self.rpy_limits['pitch'],
                'deg': (np.degrees(self.rpy_limits['pitch'][0]),
                        np.degrees(self.rpy_limits['pitch'][1]))
            },
            'yaw': {
                'rad': self.rpy_limits['yaw'],
                'deg': (np.degrees(self.rpy_limits['yaw'][0]),
                        np.degrees(self.rpy_limits['yaw'][1]))
            }
        }

    def print_kinematics_info(self,
                              roll: float,
                              pitch: float,
                              yaw: float):
        """
        打印运动学信息 (调试用)

        参数:
            roll, pitch, yaw: RPY角度 (弧度)
        """
        print("\n=== 脚踝运动学信息 ===")
        print(f"输入 RPY (度): R={np.degrees(roll):.2f}°, "
              f"P={np.degrees(pitch):.2f}°, Y={np.degrees(yaw):.2f}°")

        result = self.rpy_to_theta_angles(roll, pitch, yaw)

        print(f"\nTheta角 (连杆与β平面夹角):")
        print(f"  θ_a = {np.degrees(result['theta_a']):.2f}°")
        print(f"  θ_b = {np.degrees(result['theta_b']):.2f}°")
        print(f"  θ_c = {np.degrees(result['theta_c']):.2f}°")

        print(f"\nPhi角 (连杆与z轴夹角):")
        print(f"  φ_a = {np.degrees(result['phi_a']):.2f}°")
        print(f"  φ_b = {np.degrees(result['phi_b']):.2f}°")
        print(f"  φ_c = {np.degrees(result['phi_c']):.2f}°")

        print(f"\n连杆长度: {result['rod_lengths']}")
        print(f"约束满足: {'是' if result['constraint_satisfied'] else '否'}")
        print(f"约束误差: {result['constraint_norm']:.2e}")

        commands = self.rpy_to_servo_commands(roll, pitch, yaw)
        print(f"\n舵机命令:")
        for cmd in commands:
            print(f"  ID={cmd['id']}: position={cmd['position']}us, "
                  f"theta={cmd['theta_deg']:.2f}°")
        print("=" * 40 + "\n")


# 示例和测试代码
if __name__ == '__main__':
    print("3-DOF并联机构运动学求解器测试\n")

    # 创建求解器实例
    solver = Parallel3DOFKinematicsSolver(l0=0.02, l1=0.02, l2=0.02)

    # 测试用例1: 零姿态
    print("测试用例1: 零姿态 (0, 0, 0)")
    solver.print_kinematics_info(0, 0, 0)

    # 测试用例2: 小角度姿态
    print("测试用例2: 小角度姿态 (10°, 10°, 5°)")
    solver.print_kinematics_info(
        np.radians(10),
        np.radians(10),
        np.radians(5)
    )

    # 测试用例3: 边界姿态
    print("测试用例3: 边界姿态 (30°, 30°, 0°)")
    solver.print_kinematics_info(
        np.radians(30),
        np.radians(30),
        np.radians(0)
    )

    # 打印工作空间限制
    print("\n工作空间限制:")
    limits = solver.get_workspace_limits()
    for axis, lim in limits.items():
        print(f"  {axis}: {lim['deg'][0]:.1f}° ~ {lim['deg'][1]:.1f}°")
