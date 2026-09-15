"""平面2R机械臂的XY位置雅可比矩阵"""
import numpy as np

from armlab._validation import finite_scalars, positive_scalars


def jacobian(q1, q2, a1, a2):
    """返回一个2×2雅可比矩阵：J @ q_dot 将弧度/秒映射为米/秒"""
    finite_scalars(q1=q1, q2=q2)
    positive_scalars(a1=a1, a2=a2)
    return np.array([
        [-a1 * np.sin(q1) - a2 * np.sin(q1 + q2), -a2 * np.sin(q1 + q2)],
        [a1 * np.cos(q1) + a2 * np.cos(q1 + q2), a2 * np.cos(q1 + q2)],
    ])
