"""
3自由度并联机构通用控制器

基于3-DOF并联机构运动学的通用控制器，可用于脚踝、腰部等关节
"""

from .kinematics_solver import Parallel3DOFKinematicsSolver

__all__ = ['Parallel3DOFKinematicsSolver']
__version__ = '0.1.0'
