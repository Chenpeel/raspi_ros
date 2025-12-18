import numpy as np
try:
    from scipy.optimize import fsolve
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("注意: scipy未安装，inverse_kinematics_scipy方法将不可用")
from typing import Tuple, Dict, Optional


class ThreeDOFParallelMechanism:
    """
    三自由度并联机构运动学求解器
    3-DOF parallel mechanism
    """

    def __init__(self, l0: float, l1: float, l2: float):
        """
        参数:
            l0: 平台半径
            l1: 动平台到O点的距离
            l2: 静平台到O点的距离
        """
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2

        # 动平台点（在α平面）
        self.Mu = self._construct_platform_points(l0, -l1)
        # 静平台点初始位置（在β平面，δ=-π时）
        self.Nu = self._construct_platform_points(l0, l2)

    @staticmethod
    def _construct_platform_points(l0: float, z: float) -> np.ndarray:
        """构造正三角形平台的三个顶点"""
        sqrt3 = np.sqrt(3)
        return np.array([
            [0, -l0, z],                    # A 或 A'
            [-sqrt3/2*l0, l0/2, z],         # B 或 B'
            [sqrt3/2*l0, l0/2, z]           # C 或 C'
        ]).T

    @staticmethod
    def rotation_matrix(mu: np.ndarray) -> np.ndarray:
        """
        构造ZYX欧拉角旋转矩阵

        参数:
            mu: [roll, pitch, yaw] 姿态角向量

        返回:
            3x3旋转矩阵 R(μ) = Rz(y) · Ry(p) · Rx(r)

        性质:
            - 正交矩阵：R^T R = I，det(R) = 1
            - 保持向量模长：|R·v| = |v|
            - 保持向量夹角：⟨R·u, R·v⟩ = ⟨u, v⟩

        注意:
            - 旋转顺序：先绕x轴（roll），再绕y轴（pitch），最后绕z轴（yaw）
            - 与文档res.md第1.2节完全一致
        """
        r, p, y = mu

        # 预计算三角函数
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)

        # R = Rz(y) * Ry(p) * Rx(r)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ])

        return R

    @staticmethod
    def rotation_matrix_derivatives(mu: np.ndarray) -> Tuple[np.ndarray, np.ndarray,
                                                             np.ndarray]:
        """
        计算旋转矩阵对各姿态角的偏导数

        参数:
            mu: [roll, pitch, yaw] 姿态角向量

        返回:
            (dR/dr, dR/dp, dR/dy) 三个3x3偏导数矩阵

        说明:
            - dR/dr: 旋转矩阵对roll角的偏导数
            - dR/dp: 旋转矩阵对pitch角的偏导数
            - dR/dy: 旋转矩阵对yaw角的偏导数

        用途:
            - 雅可比矩阵计算（逆向运动学）
            - 速度运动学分析

        注意:
            - 与文档res.md第2.5节完全一致
        """
        r, p, y = mu

        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)

        # ∂R/∂r
        dR_dr = np.array([
            [0, cy*sp*cr + sy*sr, -cy*sp*sr + sy*cr],
            [0, sy*sp*cr - cy*sr, -sy*sp*sr - cy*cr],
            [0, cp*cr,            -cp*sr]
        ])

        # ∂R/∂p
        dR_dp = np.array([
            [-cy*sp, cy*cp*sr, cy*cp*cr],
            [-sy*sp, sy*cp*sr, sy*cp*cr],
            [-cp,    -sp*sr,   -sp*cr]
        ])

        # ∂R/∂y
        dR_dy = np.array([
            [-sy*cp, -sy*sp*sr - cy*cr, -sy*sp*cr + cy*sr],
            [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
            [0,      0,                  0]
        ])

        return dR_dr, dR_dp, dR_dy

    def forward_kinematics(self, mu: np.ndarray, mu_z: float = 0.0) -> Dict:
        """
        正向运动学：从姿态角求解点位置和杆角度

        参数:
            mu: [roll, pitch, yaw] 姿态角，范围 [-π/6, π/6]
            mu_z: β平面绕其法向量的额外旋转参数（通常设为0）
                  物理意义：用于建模静平台内部的旋转偏差或制造误差

        返回:
            包含A', B', C'坐标和各杆角度的字典

        注意:
            - 当 mu_z = 0 时，使用标准推导：Nu' = R(μ) · Nu
            - 当 mu_z ≠ 0 时，在β平面内额外旋转（实验性功能）
            - 并非所有姿态角都满足约束，只有通过逆向运动学求解的姿态角才满足
            - 使用 verify_constraints() 方法检查约束是否满足
        """
        # 1. 计算旋转矩阵
        R = self.rotation_matrix(mu)

        # 2. 计算β平面法向量
        n_beta = R @ np.array([0, 0, 1])

        # 3. 计算静平台点的旋转后位置
        # 标准情况（mu_z = 0）：直接旋转 Nu' = R(μ) · Nu
        if mu_z == 0.0:
            Nu_prime = R @ self.Nu
        else:
            # 实验性功能：在β平面内额外旋转
            # 使用Rodrigues旋转公式绕n_beta旋转mu_z角度
            K = self._skew_symmetric(n_beta)
            R_beta = np.eye(3) + np.sin(mu_z) * K + (1 - np.cos(mu_z)) * K @ K
            Nu_prime = R_beta @ (R @ self.Nu)

        # 4. 计算连杆向量和长度
        rods = Nu_prime - self.Mu
        rod_lengths = np.linalg.norm(rods, axis=0)

        # 5. 计算L₂位置（用于输出）
        OL2 = self.l2 * n_beta

        # 6. 计算角度
        z_axis = np.array([0, 0, 1])

        angles = {}
        labels = ['a', 'b', 'c']

        for i, label in enumerate(labels):
            rod = rods[:, i]
            rod_unit = rod / rod_lengths[i]

            # φ: 杆与z轴(OL1方向)的夹角
            phi = np.arccos(np.clip(rod_unit @ z_axis, -1, 1))

            # θ: 杆与β平面的夹角
            # 杆与法向量的夹角的余角
            cos_angle = np.clip(rod_unit @ n_beta, -1, 1)
            theta = np.pi/2 - np.arccos(cos_angle)

            angles[f'phi_{label}'] = phi
            angles[f'theta_{label}'] = theta

        return {
            'A_prime': Nu_prime[:, 0],
            'B_prime': Nu_prime[:, 1],
            'C_prime': Nu_prime[:, 2],
            'rod_lengths': rod_lengths,
            'n_beta': n_beta,
            'OL2': OL2,
            **angles
        }

    @staticmethod
    def _skew_symmetric(v: np.ndarray) -> np.ndarray:
        """构造向量的斜对称矩阵"""
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

    def compute_jacobian(self, mu: np.ndarray) -> np.ndarray:
        """
        计算雅可比矩阵 J = ∂F/∂μ

        参数:
            mu: 当前姿态角 [r, p, y]

        返回:
            3x3雅可比矩阵

        数学定义:
            J_ij = ∂f_i/∂μ_j = η_i^T (∂R/∂μ_j)^T γ_i

        物理意义:
            - 描述姿态角变化如何影响约束方程
            - 用于Newton-Raphson逆向运动学求解
            - 奇异性检测：det(J) = 0 或 cond(J) > 10^10

        注意:
            - 与文档res.md第2.4节完全一致
        """
        dR_dr, dR_dp, dR_dy = self.rotation_matrix_derivatives(mu)

        J = np.zeros((3, 3))

        for i in range(3):
            eta_i = self.Nu[:, i]
            gamma_i = self.Mu[:, i]

            # ∂f_i/∂r
            J[i, 0] = eta_i.T @ dR_dr.T @ gamma_i

            # ∂f_i/∂p
            J[i, 1] = eta_i.T @ dR_dp.T @ gamma_i

            # ∂f_i/∂y
            J[i, 2] = eta_i.T @ dR_dy.T @ gamma_i

        return J

    def constraint_equations(self, mu: np.ndarray) -> np.ndarray:
        """
        约束方程 F(μ) = 0

        F_i = (η'_i - γ_i) · γ_i = 0

        物理意义：连杆垂直于动平台半径（核心约束条件）

        参数:
            mu: 姿态角 [r, p, y]

        返回:
            约束方程值向量 [f_1, f_2, f_3]

        注意:
            - 当 ||F(μ)|| < 1e-6 时，认为约束满足
            - 此约束与文档res.md第1.5.1节完全一致
        """
        R = self.rotation_matrix(mu)
        Nu_prime = R @ self.Nu

        F = np.zeros(3)
        for i in range(3):
            rod = Nu_prime[:, i] - self.Mu[:, i]
            F[i] = rod @ self.Mu[:, i]

        return F

    def inverse_kinematics_newton(self,
                                  target_angles: Optional[Dict[str, float]] = None,
                                  target_positions: Optional[np.ndarray] = None,
                                  mu_init: Optional[np.ndarray] = None,
                                  max_iter: int = 50,
                                  tol: float = 1e-6) -> Dict:
        """
        逆向运动学：Newton-Raphson迭代法

        参数:
            target_angles: 目标角度字典（暂未使用）
            target_positions: 目标位置3x3数组（暂未使用）
            mu_init: 初始猜测姿态角，默认为零姿态
            max_iter: 最大迭代次数
            tol: 收敛容差

        返回:
            求解结果字典，包含：
            - mu: 求解得到的姿态角
            - iterations: 迭代次数
            - converged: 是否收敛
            - 其他正向运动学结果

        算法:
            μ^(k+1) = μ^(k) - J^(-1)(μ^(k)) · F(μ^(k))

        收敛判据:
            ||F(μ)|| < tol

        注意:
            - 初始猜测影响收敛速度和结果
            - 检测奇异性（cond(J) > 10^10）
            - 自动限制在工作空间 [-π/6, π/6] 内
            - 与文档res.md第2.7节完全一致
        """
        if mu_init is None:
            mu_init = np.zeros(3)

        mu = mu_init.copy()

        for iteration in range(max_iter):
            # 计算约束方程值
            F = self.constraint_equations(mu)

            # 检查收敛
            if np.linalg.norm(F) < tol:
                result = self.forward_kinematics(mu)
                result['mu'] = mu
                result['iterations'] = iteration
                result['converged'] = True
                return result

            # 计算雅可比矩阵
            J = self.compute_jacobian(mu)

            # 检查奇异性
            if np.linalg.cond(J) > 1e10:
                print(f"警告：雅可比矩阵接近奇异，条件数 = {np.linalg.cond(J)}")

            # Newton-Raphson更新
            try:
                delta_mu = np.linalg.solve(J, -F)
            except np.linalg.LinAlgError:
                print("雅可比矩阵奇异，使用伪逆")
                delta_mu = np.linalg.pinv(J) @ (-F)

            # 更新姿态角
            mu = mu + delta_mu

            # 限制在工作空间内
            mu = np.clip(mu, -np.pi/6, np.pi/6)

        print(f"警告：未收敛，最终误差 = {np.linalg.norm(F)}")
        result = self.forward_kinematics(mu)
        result['mu'] = mu
        result['iterations'] = max_iter
        result['converged'] = False
        return result

    def inverse_kinematics_scipy(self,
                                 target_positions: Optional[np.ndarray] = None,
                                 mu_init: Optional[np.ndarray] = None) -> Dict:
        """
        使用scipy的fsolve求解逆运动学

        参数:
            target_positions: 目标位置3x3数组（如果提供，求解满足该位置的姿态角）
            mu_init: 初始猜测姿态角，默认为零姿态

        返回:
            求解结果字典，包含：
            - mu: 求解得到的姿态角
            - converged: 是否收敛（ier == 1）
            - message: scipy求解器返回的消息
            - 其他正向运动学结果

        说明:
            - 如果提供 target_positions，求解 Nu' = target_positions
            - 否则求解约束方程 F(μ) = 0
            - 使用scipy.optimize.fsolve进行求解
        """
        if not SCIPY_AVAILABLE:
            raise ImportError("此方法需要scipy库，请安装: pip install scipy")

        if mu_init is None:
            mu_init = np.zeros(3)

        # 定义目标函数（如果提供了目标位置）
        if target_positions is not None:
            def objective(mu):
                R = self.rotation_matrix(mu)
                Nu_prime = R @ self.Nu
                return (Nu_prime - target_positions).flatten()
        else:
            objective = self.constraint_equations

        # 求解
        mu_solution, infodict, ier, msg = fsolve(
            objective, mu_init, full_output=True)

        result = self.forward_kinematics(mu_solution)
        result['mu'] = mu_solution
        result['converged'] = (ier == 1)
        result['message'] = msg

        return result

    def velocity_kinematics(self, mu: np.ndarray, mu_dot: np.ndarray) -> Dict:
        """
        速度运动学：计算静平台点的速度

        参数:
            mu: 当前姿态角 [r, p, y]
            mu_dot: 姿态角速度 [ṙ, ṗ, ẏ]

        返回:
            速度字典，包含：
            - F_dot: 约束方程时间导数 J·μ̇
            - A_prime_dot: A'点的速度向量
            - B_prime_dot: B'点的速度向量
            - C_prime_dot: C'点的速度向量
            - jacobian: 雅可比矩阵

        数学原理:
            η̇'ᵢ = Ṙ(μ) · ηᵢ
            其中 Ṙ = (∂R/∂r)ṙ + (∂R/∂p)ṗ + (∂R/∂y)ẏ

        注意:
            - 与文档res.md第3.3节完全一致
            - 约束空间速度满足 J·μ̇ = 0（约束保持）
        """
        J = self.compute_jacobian(mu)

        # 计算约束空间的速度
        F_dot = J @ mu_dot

        # 计算旋转矩阵时间导数
        dR_dr, dR_dp, dR_dy = self.rotation_matrix_derivatives(mu)
        R_dot = (dR_dr * mu_dot[0] + dR_dp * mu_dot[1] + dR_dy * mu_dot[2])

        # 计算端点速度
        Nu_prime_dot = R_dot @ self.Nu

        return {
            'F_dot': F_dot,
            'A_prime_dot': Nu_prime_dot[:, 0],
            'B_prime_dot': Nu_prime_dot[:, 1],
            'C_prime_dot': Nu_prime_dot[:, 2],
            'jacobian': J
        }

    def verify_constraints(self, mu: np.ndarray, tol: float = 1e-6) -> Dict:
        """
        验证给定姿态角是否满足垂直约束

        参数:
            mu: 姿态角 [r, p, y]
            tol: 容差阈值

        返回:
            验证结果字典，包含：
            - satisfied: 是否满足约束（布尔值）
            - constraint_values: 约束方程值 [f_1, f_2, f_3]
            - constraint_norm: ||F(μ)||
            - individual_satisfied: 各杆约束是否满足 [bool, bool, bool]

        判据:
            - 满足约束：||F(μ)|| < tol
            - 约束方程：(η'ᵢ - γᵢ) · γᵢ = 0

        注意:
            - 与文档res.md第1.5.1节约束定义一致
        """
        F = self.constraint_equations(mu)
        F_norm = np.linalg.norm(F)

        return {
            'satisfied': F_norm < tol,
            'constraint_values': F,
            'constraint_norm': F_norm,
            'individual_satisfied': [abs(f) < tol for f in F]
        }

    def compute_theoretical_rod_length(self) -> float:
        """
        计算理论杆长（满足约束时）

        返回:
            理论杆长 s = √(l₂² - l₁²)

        推导:
            由约束 (η'ᵢ - γᵢ) · γᵢ = 0 和 |η'ᵢ| = √(l₀² + l₂²)
            可得 sᵢ = √(l₂² - l₁²)

        注意:
            - 三根杆长度相等：s₁ = s₂ = s₃
            - 奇异条件：l₁ = l₂ 时杆长为0
            - 与文档res.md第1.7节完全一致
        """
        return np.sqrt(self.l2**2 - self.l1**2)

    def verify_rod_lengths(self, mu: np.ndarray, tol: float = 1e-6) -> Dict:
        """
        验证实际杆长是否符合理论值

        参数:
            mu: 姿态角 [r, p, y]
            tol: 容差阈值

        返回:
            验证结果字典，包含：
            - satisfied: 是否满足（所有杆长误差 < tol）
            - theoretical_length: 理论杆长
            - actual_lengths: 实际杆长 [s_a, s_b, s_c]
            - errors: 误差 [e_a, e_b, e_c]
            - max_error: 最大误差

        说明:
            - 在满足约束的情况下，三根杆长应相等且等于理论值
            - 实际应用中可能因制造误差或数值误差产生偏差
        """
        theoretical = self.compute_theoretical_rod_length()
        result_fk = self.forward_kinematics(mu)
        actual_lengths = result_fk['rod_lengths']
        errors = np.abs(actual_lengths - theoretical)

        return {
            'satisfied': np.all(errors < tol),
            'theoretical_length': theoretical,
            'actual_lengths': actual_lengths,
            'errors': errors,
            'max_error': np.max(errors)
        }
